#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "map/terrain_surface_fit.hpp"

using namespace lunar::incremental_navigation::map;

namespace {
std::vector<SurfaceSample> Patch(bool curved = false) {
  std::vector<SurfaceSample> samples;
  for (int y = -4; y <= 4; ++y) {
    for (int x = -4; x <= 4; ++x) {
      const double px = x * .05, py = y * .05;
      const double z = .7 + .3 * px - .2 * py +
                       (curved ? 2. * px * px + .5 * px * py - py * py : 0.);
      samples.push_back({px, py, z, z, z});
    }
  }
  return samples;
}

TEST(TerrainWeightedPlane, PreservesExactSlopeAndAbsoluteHeight) {
  const auto fit = FitWeightedPlane(Patch(), {});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->Height(0., 0.), .7, 1e-12);
  EXPECT_NEAR(fit->Gradient(0., 0.).x, .3, 1e-12);
  EXPECT_NEAR(fit->Gradient(0., 0.).y, -.2, 1e-12);
  EXPECT_NEAR(fit->Height(.1, -.1), .75, 1e-12);
  EXPECT_EQ(std::count(fit->inliers.begin(), fit->inliers.end(), true), 81);
  EXPECT_FALSE(fit->used_fallback);
}

TEST(TerrainWeightedPlane, RejectsCollinearOrNonFiniteEvidence) {
  std::vector<SurfaceSample> samples;
  for (int i = -4; i <= 4; ++i) samples.push_back({i*.05, i*.05, 0., 0., 0.});
  EXPECT_FALSE(FitWeightedPlane(samples, {}));
  samples = Patch();
  samples[3].z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(FitWeightedPlane(samples, {}));
  EXPECT_FALSE(FitWeightedPlane(Patch(), {.support_distance_m=0.}));
}

TEST(TerrainWeightedPlane, HandlesNoiseAndRejectsLargeOutliers) {
  auto samples = Patch();
  std::mt19937 random(4105);
  std::normal_distribution<double> noise(0., .006);
  for (auto& sample : samples) {
    const double perturbation = noise(random);
    sample.z += perturbation;
    sample.low = sample.high = sample.z;
  }
  for (std::size_t i : {0u, 12u, 23u, 35u, 48u, 62u, 75u}) {
    samples[i].z += .25;
    samples[i].low = samples[i].high = samples[i].z;
  }
  const auto fit = FitWeightedPlane(samples, {});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->Height(0., 0.), .7, .004);
  EXPECT_NEAR(fit->Gradient(0., 0.).x, .3, .025);
  EXPECT_NEAR(fit->Gradient(0., 0.).y, -.2, .025);
  for (std::size_t i : {0u, 12u, 23u, 35u, 48u, 62u, 75u}) EXPECT_FALSE(fit->inliers[i]);
}

TEST(TerrainWeightedPlane, WideRawIntervalsReceiveLessInfluence) {
  auto narrow = Patch();
  for (auto& sample : narrow) if (sample.x > .05) sample.z += .02;
  for (auto& sample : narrow) sample.low = sample.high = sample.z;
  auto wide = narrow;
  for (auto& sample : wide) if (sample.x > .05) {
    sample.low = sample.z - .15;
    sample.high = sample.z + .15;
  }
  const auto narrow_fit = FitWeightedPlane(narrow, {});
  const auto wide_fit = FitWeightedPlane(wide, {});
  ASSERT_TRUE(narrow_fit);
  ASSERT_TRUE(wide_fit);
  EXPECT_LT(std::abs(wide_fit->Height(0., 0.) - .7),
            std::abs(narrow_fit->Height(0., 0.) - .7));
  EXPECT_LT(std::abs(wide_fit->Gradient(0., 0.).x - .3),
            std::abs(narrow_fit->Gradient(0., 0.).x - .3));
}

TEST(TerrainWeightedPlane, ShorterDistanceKernelReducesRemoteEvidenceInfluence) {
  auto samples = Patch();
  for (auto& sample : samples) {
    if (std::hypot(sample.x, sample.y) >= .18) sample.z += .012;
    sample.low = sample.high = sample.z;
  }
  const auto broad = FitWeightedPlane(samples, {.support_distance_m=.2});
  const auto local = FitWeightedPlane(samples, {.support_distance_m=.1});
  ASSERT_TRUE(broad);
  ASSERT_TRUE(local);
  EXPECT_LT(std::abs(local->Height(0., 0.) - .7),
            std::abs(broad->Height(0., 0.) - .7));
}

TEST(TerrainMovingLeastSquares, PreservesAnExactQuadraticAndItsGradient) {
  const auto fit = FitMovingLeastSquares(Patch(true), {});
  ASSERT_TRUE(fit);
  EXPECT_FALSE(fit->used_fallback);
  EXPECT_NEAR(fit->Height(0., 0.), .7, 1e-12);
  EXPECT_NEAR(fit->Height(.08, -.06), .7428, 1e-12);
  EXPECT_NEAR(fit->Gradient(.08, -.06).x, .59, 1e-12);
  EXPECT_NEAR(fit->Gradient(.08, -.06).y, -.04, 1e-12);
  EXPECT_NEAR(fit->coefficients[3], 2., 1e-11);
  EXPECT_NEAR(fit->coefficients[4], .5, 1e-11);
  EXPECT_NEAR(fit->coefficients[5], -1., 1e-11);
}

TEST(TerrainMovingLeastSquares, ImprovesCurvedSurfaceCenterBiasOverPlane) {
  const auto samples = Patch(true);
  const auto plane = FitWeightedPlane(samples, {});
  const auto quadratic = FitMovingLeastSquares(samples, {});
  ASSERT_TRUE(plane);
  ASSERT_TRUE(quadratic);
  EXPECT_GT(std::abs(plane->Height(0., 0.) - .7), .005);
  EXPECT_LT(std::abs(quadratic->Height(0., 0.) - .7), 1e-10);
}

TEST(TerrainMovingLeastSquares, SparseOrRankDeficientQuadraticsFallBackToPlane) {
  std::vector<SurfaceSample> corners;
  for (double x : {-.1, .1}) for (double y : {-.1, .1}) {
    const double z = .7 + .3*x - .2*y;
    corners.push_back({x, y, z, z, z});
  }
  auto fit = FitMovingLeastSquares(corners, {});
  ASSERT_TRUE(fit);
  EXPECT_TRUE(fit->used_fallback);
  EXPECT_NEAR(fit->Height(0., 0.), .7, 1e-12);
  corners.clear();
  // More than six samples, but only two distinct x coordinates: x*x cannot
  // be identified separately from the constant coefficient.
  for (double x : {-.1, .1}) for (int i = -4; i <= 4; ++i) {
    const double y = i*.025, z = .7 + .3*x - .2*y;
    corners.push_back({x, y, z, z, z});
  }
  fit = FitMovingLeastSquares(corners, {});
  ASSERT_TRUE(fit);
  EXPECT_TRUE(fit->used_fallback);
  EXPECT_NEAR(fit->Gradient(0., 0.).x, .3, 1e-12);
  corners.clear();
  for (int i = -4; i <= 4; ++i) {
    const double x = i*.025, z = .7+.3*x;
    corners.push_back({x, x, z, z, z});
  }
  EXPECT_FALSE(FitMovingLeastSquares(corners, {}));
}

TEST(TerrainMovingLeastSquares, RobustCurvedFitHandlesNoiseAndOutliers) {
  auto samples = Patch(true);
  std::mt19937 random(9011);
  std::normal_distribution<double> noise(0., .003);
  for (auto& sample : samples) {
    sample.z += noise(random);
    sample.low = sample.high = sample.z;
  }
  for (std::size_t i : {3u, 16u, 29u, 52u, 65u, 79u}) {
    samples[i].z -= .3;
    samples[i].low = samples[i].high = samples[i].z;
  }
  const auto fit = FitMovingLeastSquares(samples, {});
  ASSERT_TRUE(fit);
  EXPECT_FALSE(fit->used_fallback);
  EXPECT_NEAR(fit->Height(0., 0.), .7, .004);
  EXPECT_NEAR(fit->Gradient(0., 0.).x, .3, .025);
  EXPECT_NEAR(fit->Gradient(0., 0.).y, -.2, .025);
  for (std::size_t i : {3u, 16u, 29u, 52u, 65u, 79u}) EXPECT_FALSE(fit->inliers[i]);
}
}  // namespace
