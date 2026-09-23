#include "local/boundary_distance_index.hpp"
#include <gtest/gtest.h>
#include <random>
#include <stop_token>

namespace lunar::incremental_navigation::local {
namespace {
// Independent exhaustive oracle, retained from the pre-index segment metric.
long double PointDistance(BoundaryPoint p, BoundarySegment s) {
  const auto dx = s.b.x - s.a.x, dy = s.b.y - s.a.y;
  const auto d = dx * dx + dy * dy;
  auto t = d > 0 ? std::clamp(((p.x - s.a.x) * dx + (p.y - s.a.y) * dy) / d,
                              0.L, 1.L)
                 : 0.L;
  auto x = p.x - s.a.x - t * dx, y = p.y - s.a.y - t * dy;
  return x * x + y * y;
}
long double Exhaustive(const std::vector<BoundarySegment> &edges,
                       BoundarySegment q) {
  auto best = std::numeric_limits<long double>::infinity();
  for (auto e : edges)
    best = std::min({best, PointDistance(q.a, e), PointDistance(q.b, e),
                     PointDistance(e.a, q), PointDistance(e.b, q)});
  return best;
}
TEST(BoundaryDistanceIndex, ExactKnownDistancesAndEmptyBoundary) {
  SearchControl control;
  BoundaryDistanceIndex index;
  ASSERT_TRUE(index.Build({}, control));
  ASSERT_TRUE(index.Query({{0, 0}, {1, 1}}, control));
  EXPECT_TRUE(std::isinf(*index.Query({{0, 0}, {1, 1}}, control)));
  ASSERT_TRUE(index.Build({{{0, 0}, {10, 0}}, {{10, 0}, {10, 10}}}, control));
  EXPECT_EQ(*index.Query({{2, 3}, {6, 3}}, control), 9.L);
  EXPECT_EQ(*index.Query({{10, 10}, {10, 10}}, control), 0.L);
}
TEST(BoundaryDistanceIndex,
     SeededDisjointSegmentsMatchExhaustiveDistanceAndReversal) {
  std::mt19937 rng(901);
  SearchControl control;
  for (int fixture = 0; fixture < 50; ++fixture) {
    std::vector<BoundarySegment> edges;
    const long double shift = fixture % 2 ? 1.e12L : 0.L;
    for (int i = 0; i < 129 + fixture; ++i) {
      long double x = shift + (rng() % 1000) / 10.L,
                  y = shift + (rng() % 1000) / 10.L;
      long double length = (rng() % 101) / 10.L;
      edges.push_back({{x, y},
                       i % 2 ? BoundaryPoint{x + length, y}
                             : BoundaryPoint{x, y + length}});
    }
    BoundaryDistanceIndex index;
    ASSERT_TRUE(index.Build(edges, control));
    for (int i = 0; i < 100; ++i) {
      BoundarySegment q{
          {shift - 1 - (rng() % 100) / 10.L, shift + (rng() % 1000) / 10.L},
          {shift - 1 - (rng() % 100) / 10.L, shift + (rng() % 1000) / 10.L}};
      auto actual = index.Query(q, control);
      ASSERT_TRUE(actual);
      EXPECT_EQ(*actual, Exhaustive(edges, q));
      std::swap(q.a, q.b);
      auto reverse = index.Query(q, control);
      ASSERT_TRUE(reverse);
      EXPECT_EQ(*reverse, Exhaustive(edges, q));
    }
  }
}
TEST(BoundaryDistanceIndex, InteriorPointsAndNearTiesMatchExhaustiveMetric) {
  SearchControl control;
  std::vector<BoundarySegment> edges;
  for (int y = 0; y < 30; ++y)
    for (int x = 0; x < 30; ++x)
      edges.push_back({{x * .1L, y * .1L}, {x * .1L + .075L, y * .1L}});
  BoundaryDistanceIndex index;
  ASSERT_TRUE(index.Build(edges, control));
  std::mt19937 rng(773);
  for (int i = 0; i < 5000; ++i) {
    const long double x = (rng() % 30000) / 10000.L;
    const long double y = (rng() % 30000) / 10000.L;
    BoundarySegment point{{x, y}, {x, y}};
    ASSERT_TRUE(index.Query(point, control));
    EXPECT_EQ(*index.Query(point, control), Exhaustive(edges, point));
  }
  int ticks = 0;
  SearchControl interrupted{
      .deadline = SteadyClock::time_point{} + std::chrono::seconds(3),
      .now = [&] {
        return SteadyClock::time_point{} + std::chrono::seconds(++ticks);
      }};
  EXPECT_FALSE(index.Query({{1.01L, 1.01L}, {1.02L, 1.01L}}, interrupted));
}

TEST(BoundaryDistanceIndex, StopsDuringBuildAndQuery) {
  std::vector<BoundarySegment> edges;
  for (int i = 0; i < 10000; ++i)
    edges.push_back({{(long double)i, 0}, {(long double)i, 10}});
  BoundaryDistanceIndex index;
  SearchControl normal;
  std::stop_source stop;
  stop.request_stop();
  SearchControl canceled{.stop_token = stop.get_token()};
  EXPECT_FALSE(index.Build(edges, canceled));
  EXPECT_FALSE(index.Query({{-2, 1}, {-1, 2}}, normal));
  ASSERT_TRUE(index.Build(edges, normal));
  EXPECT_FALSE(index.Query({{-2, 1}, {-1, 2}}, canceled));
  SearchControl expired{.deadline = SteadyClock::time_point::min()};
  EXPECT_FALSE(index.Query({{-2, 1}, {-1, 2}}, expired));
  int ticks = 0;
  SearchControl during{
      .deadline = SteadyClock::time_point{} + std::chrono::seconds(10),
      .now = [&] {
        return SteadyClock::time_point{} + std::chrono::seconds(++ticks);
      }};
  EXPECT_FALSE(index.Build(edges, during));
  EXPECT_GE(ticks, 10);
}
} // namespace
} // namespace lunar::incremental_navigation::local
