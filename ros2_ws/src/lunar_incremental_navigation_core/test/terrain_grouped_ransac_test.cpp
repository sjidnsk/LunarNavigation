#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "map/terrain_surface_fit.hpp"

using namespace lunar::incremental_navigation::map;

namespace {
void AddPlane(std::vector<SurfaceSample>& samples, int lower, int upper,
              double x_offset, double y_offset, double c, double a, double b,
              double spacing = .05) {
  for (int y = lower; y <= upper; ++y) {
    for (int x = lower; x <= upper; ++x) {
      const double px = x_offset + x * spacing;
      const double py = y_offset + y * spacing;
      const double z = c + a * px + b * py;
      samples.push_back({px, py, z, z, z});
    }
  }
}
double OriginHeight(const SurfaceFit& fit) { return fit.coefficients[0]; }
}  // namespace

TEST(TerrainGroupedRansac, PlaneWithIsolatedOutliersKeepsSlope) {
  std::vector<SurfaceSample> samples;
  AddPlane(samples, -2, 2, 0., 0., .2, .4, -.2);
  for (std::size_t i : {0u, 6u, 24u}) {
    samples[i].low += .35;
    samples[i].high += .35;
    samples[i].z += .35;
  }
  auto fit = FitGroupedRansac(samples, {});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(OriginHeight(*fit), .2, 1e-10);
  EXPECT_NEAR(fit->coefficients[1], .4, 1e-10);
  EXPECT_NEAR(fit->coefficients[2], -.2, 1e-10);
  ASSERT_EQ(fit->inliers.size(), samples.size());
  EXPECT_FALSE(fit->inliers[0]);
  EXPECT_FALSE(fit->inliers[6]);
  EXPECT_FALSE(fit->inliers[24]);
}

TEST(TerrainGroupedRansac, LocalSupportedPlaneWinsOverLargerDistantPlane) {
  std::vector<SurfaceSample> samples;
  AddPlane(samples, -1, 1, 0., 0., .1, .2, .1);
  AddPlane(samples, -3, 3, .5, 0., .45, 0., 0.);
  auto fit = FitGroupedRansac(samples, {});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(OriginHeight(*fit), .1, 1e-10);
  EXPECT_NEAR(fit->coefficients[1], .2, 1e-10);
  for (std::size_t i = 9; i < samples.size(); ++i) EXPECT_FALSE(fit->inliers[i]);
}

TEST(TerrainGroupedRansac, RemoteStepDoesNotFragmentTheSupportedTargetPlane) {
  std::vector<SurfaceSample> samples;
  for (int y = -6; y <= 6; ++y) for (int x = -6; x <= 6; ++x) {
    if (x == 0 && y == 0) continue;
    const double z = .2 + .4 * x * .05 - .2 * y * .05 + (x >= 3 ? .08 : 0.);
    samples.push_back({x * .05, y * .05, z, z, z});
  }
  auto fit = FitGroupedRansac(samples, {});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(OriginHeight(*fit), .2, 1e-10);
  EXPECT_NEAR(fit->coefficients[1], .4, 1e-10);
  EXPECT_NEAR(fit->coefficients[2], -.2, 1e-10);
  EXPECT_EQ(std::count(fit->inliers.begin(), fit->inliers.end(), true), 116);
}

TEST(TerrainGroupedRansac, DisconnectedCoplanarBanksDoNotBridgeGap) {
  std::vector<SurfaceSample> samples;
  AddPlane(samples, -1, 1, -.2, 0., 0., 0., 0.);
  AddPlane(samples, -1, 1, .2, 0., 0., 0., 0.);
  EXPECT_FALSE(FitGroupedRansac(samples, {}));
}

TEST(TerrainGroupedRansac, CompetingSurfacesAtOriginAreRejected) {
  std::vector<SurfaceSample> samples;
  AddPlane(samples, -2, 2, 0., 0., 0., 0., 0.);
  AddPlane(samples, -2, 2, 0., 0., .2, 0., 0.);
  EXPECT_FALSE(FitGroupedRansac(samples, {}));
}

TEST(TerrainGroupedRansac, OneSidedAndCollinearSamplesCannotExtrapolate) {
  std::vector<SurfaceSample> one_sided;
  AddPlane(one_sided, 0, 3, .05, 0., 0., .3, 0.);
  EXPECT_FALSE(FitGroupedRansac(one_sided, {}));
  std::vector<SurfaceSample> line;
  for (int x = -3; x <= 3; ++x) {
    const double p = x * .05;
    line.push_back({p, p, p, p, p});
  }
  EXPECT_FALSE(FitGroupedRansac(line, {}));
}

TEST(TerrainGroupedRansac, NearbySparseSamplesCanSurroundOrigin) {
  std::vector<SurfaceSample> samples;
  for (auto xy : std::vector<std::pair<double, double>>{
           {-.075, -.075}, {-.075, .075}, {.075, -.075}, {.075, .075}}) {
    const double z = .4 * xy.first - .2 * xy.second;
    samples.push_back({xy.first, xy.second, z, z, z});
  }
  auto fit = FitGroupedRansac(samples, {});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(OriginHeight(*fit), 0., 1e-10);
  EXPECT_NEAR(fit->coefficients[1], .4, 1e-10);
}

TEST(TerrainGroupedRansac, RepeatAndInputOrderProduceSameModel) {
  std::vector<SurfaceSample> samples;
  AddPlane(samples, -2, 2, 0., 0., .2, .4, -.2);
  samples[3].low += .4;
  samples[3].high += .4;
  samples[3].z += .4;
  auto first = FitGroupedRansac(samples, {});
  auto repeat = FitGroupedRansac(samples, {});
  ASSERT_TRUE(first);
  ASSERT_TRUE(repeat);
  EXPECT_EQ(first->coefficients, repeat->coefficients);
  EXPECT_EQ(first->inliers, repeat->inliers);
  std::reverse(samples.begin(), samples.end());
  auto reverse = FitGroupedRansac(samples, {});
  ASSERT_TRUE(reverse);
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(first->coefficients[i], reverse->coefficients[i], 1e-12);
}

TEST(TerrainGroupedRansac, InvalidNumericalInputsAreRejected) {
  std::vector<SurfaceSample> samples;
  AddPlane(samples, -2, 2, 0., 0., 0., 0., 0.);
  samples[0].z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(FitGroupedRansac(samples, {}));
  samples[0].z = 0.;
  EXPECT_FALSE(FitGroupedRansac(samples, {.support_distance_m = 0.}));
  EXPECT_FALSE(FitGroupedRansac(samples, {.resolution_m = 0.}));
  EXPECT_FALSE(FitGroupedRansac(samples, {.residual_limit_m = 0.}));
}
