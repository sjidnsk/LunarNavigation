#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <random>
#include "map/influence_cells.hpp"
using namespace lunar::incremental_navigation;
namespace {
std::set<GridIndex> Oracle(const SparseGridGeometry& g,
                          const std::set<GridIndex>& changed, double radius) {
  std::set<GridIndex> result;
  for (auto cell : changed) {
    const auto [lo, hi] = detail::CellBoundsForRadius(g, cell, radius);
    for (auto y = lo.y; y < hi.y; ++y) {
      auto left = lo.x, right = hi.x;
      const auto hit = [&](auto x) {
        const Point2 center{g.origin_m().x + (static_cast<double>(x) + .5) * g.resolution_m(),
                            g.origin_m().y + (static_cast<double>(y) + .5) * g.resolution_m()};
        return CircleIntersectsCellArea(center, radius, g, cell);
      };
      while (left < right && !hit(left)) ++left;
      while (left < right && !hit(right - 1)) --right;
      for (auto x = left; x < right; ++x) result.insert({x, y});
    }
  }
  return result;
}
TEST(InfluenceRuns, DenseFrameUsesRunsRatherThanEverySourceCell) {
  SparseGridGeometry g("map", .05, {}, {-100, -100}, {100, 100});
  std::set<GridIndex> changed;
  for (int x=-40;x<40;++x) for(int y=-40;y<40;++y) changed.insert({x,y});
  detail::InfluenceStats stats;
  auto actual=detail::InfluenceCells(g, changed, .961, &stats);
  EXPECT_EQ(actual, Oracle(g,changed,.961));
  EXPECT_EQ(stats.source_runs, 80U);
  EXPECT_LT(stats.generated_spans, 4000U);
}
TEST(InfluenceRuns, MatchesOracleAcrossOffsetsRadiiAndSparseHoles) {
  std::mt19937 rng(92831);
  for (double r : {.05,.1,.2,.3}) for(double origin : {0., -.137, 1.e8, 1.e16}) {
    SparseGridGeometry g("map",r,{origin,-origin,0},{-12,-11},{13,14});
    for(double radius : {0., r*.5, std::nextafter(r*.5,0.), std::nextafter(r*.5,1.), r*2.5}) {
      std::set<GridIndex> changed;
      for(int y=-11;y<14;++y)for(int x=-12;x<13;++x)
        if((rng()%7)==0 || (y>=-2&&y<=2&&x>=-8&&x<=8))changed.insert({x,y});
      EXPECT_EQ(detail::InfluenceCells(g,changed,radius),Oracle(g,changed,radius));
    }
  }
}
TEST(InfluenceRuns, EmptyAndDisconnectedRows) {
  SparseGridGeometry g("map",.1,{}, {-10,-10},{11,11});
  EXPECT_TRUE(detail::InfluenceCells(g,{},.4).empty());
  std::set<GridIndex> changed{{-10,-10},{10,10},{-8,0},{-7,0},{0,0},{8,0},{9,0}};
  EXPECT_EQ(detail::InfluenceCells(g,changed,.4),Oracle(g,changed,.4));
}
}

TEST(InfluenceRuns, HandlesLargeIndicesAndPrecisionFallback) {
  for (std::int64_t offset : {-1000000000LL, 1000000000LL, 4503599627370490LL}) {
    for (double origin : {0., 1.e12, 1.e16}) {
      SparseGridGeometry g("map", .1, {origin, -origin, 0},
                           {offset, -8}, {offset + 21, 9});
      std::set<GridIndex> changed;
      for (auto x=offset; x<offset+21; ++x) for (int y=-3;y<=3;++y)
        if ((x-offset)%5!=0) changed.insert({x,y});
      EXPECT_EQ(detail::InfluenceCells(g,changed,.25),Oracle(g,changed,.25));
    }
  }
  SparseGridGeometry coarse("map", .1, {1.e16,0,0}, {-12,-12},{13,13});
  EXPECT_FALSE(detail::CanMergeInfluenceRuns(coarse));
  std::set<GridIndex> changed{{-13,0},{-12,0},{-11,0},{0,0},{1,0},{12,0},{13,0}};
  EXPECT_EQ(detail::InfluenceCells(coarse,changed,.4),Oracle(coarse,changed,.4));
  SparseGridGeometry normal("map", .1, {}, {-12,-12},{13,13});
  EXPECT_TRUE(detail::CanMergeInfluenceRuns(normal));
  EXPECT_EQ(detail::InfluenceCells(normal,changed,.4),Oracle(normal,changed,.4));
}
