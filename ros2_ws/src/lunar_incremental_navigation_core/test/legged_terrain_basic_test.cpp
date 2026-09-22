#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/legged_local_planner.hpp"

namespace lunar::incremental_navigation {
namespace {
struct Terrain {
  GridGeometry geometry{
      .frame_id = "map", .width = 15, .height = 15, .resolution_m = .2};
  std::vector<float> z = std::vector<float>(225, 0.F);
  PersistentElevationMap map;
  float& at(int x, int y) { return z[y * geometry.width + x]; }
  std::shared_ptr<const ElevationSnapshot> snapshot() {
    static_cast<void>(map.Apply(
        {.geometry = geometry, .elevation_m = z,
         .map_from_source = {.parent_frame = "map", .child_frame = "map"}}));
    return map.Snapshot();
  }
};
LeggedCapability Capability() {
  LeggedCapability c;
  c.maximum_slope_rad = std::numbers::pi / 6.;
  c.maximum_step_height_m = .5;
  c.maximum_gap_width_m = .3;
  c.minimum_body_clearance_m = .3;
  return c;
}
IntrinsicTraversalEvaluation Evaluate(Terrain& t, GridIndex p = {7, 7}) {
  return MakePlatformElevationEvaluator(Capability())->Evaluate(*t.snapshot(), p);
}

// Pairwise gradients turn these supported steps into artificial steep ramps.
TEST(LeggedTerrainBasic, SupportedStepsUseResidualHeightAtMultipleResolutions) {
  for (double resolution : {.1, .2})
    for (float height : {.25F, -.25F, .49F, -.49F}) {
      Terrain t;
      t.geometry.resolution_m = resolution;
      for (int y = 0; y < 15; ++y)
        for (int x = 8; x < 15; ++x) t.at(x, y) = height;
      for (GridIndex p : {GridIndex{7, 7}, GridIndex{8, 7}}) {
        const auto result = Evaluate(t, p);
        EXPECT_EQ(result.state, IntrinsicCellState::kFree);
        EXPECT_NEAR(result.relief_m, std::abs(height), 1e-5);
        EXPECT_NEAR(result.slope_rad, 0., 1e-5);
      }
    }
}

TEST(LeggedTerrainBasic, StepsOnTraversableSlopesDoNotTiltTheBackground) {
  for (double degrees : {-29., 0., 29.})
    for (float height : {-.4F, .4F}) {
      Terrain t;
      for (int y = 0; y < 15; ++y)
        for (int x = 0; x < 15; ++x)
          t.at(x, y) = float(x * .2 * std::tan(degrees * std::numbers::pi / 180.) +
                             (x >= 8 ? height : 0.));
      const auto result = Evaluate(t);
      EXPECT_EQ(result.state, IntrinsicCellState::kFree);
      EXPECT_NEAR(result.slope_rad, std::abs(degrees) * std::numbers::pi / 180., 1e-5);
      EXPECT_NEAR(result.relief_m, .4, 1e-5);
    }
}

TEST(LeggedTerrainBasic, ContinuousDiagonalSlopeIsNotAStep) {
  for (double degrees : {29., 31.}) {
    Terrain t;
    const double gradient = std::tan(degrees * std::numbers::pi / 180.) / std::sqrt(2.);
    for (int y = 0; y < 15; ++y)
      for (int x = 0; x < 15; ++x) t.at(x, y) = float((x + y) * .2 * gradient);
    const auto result = Evaluate(t);
    EXPECT_EQ(result.state, degrees < 30. ? IntrinsicCellState::kFree : IntrinsicCellState::kBlocked);
    EXPECT_NEAR(result.slope_rad, degrees * std::numbers::pi / 180., 1e-5);
    EXPECT_LT(result.relief_m, 1e-5);
  }
}

TEST(LeggedTerrainBasic, SupportedOverLimitStepsAndPitsRemainBlocked) {
  for (float height : {.51F, -.51F}) {
    Terrain t;
    for (int y = 0; y < 15; ++y)
      for (int x = 8; x < 15; ++x) t.at(x, y) = height;
    EXPECT_EQ(Evaluate(t).state, IntrinsicCellState::kBlocked);
    EXPECT_EQ(Evaluate(t, {8, 7}).state, IntrinsicCellState::kBlocked);
  }
}

TEST(LeggedTerrainBasic, IsolatedReturnsCannotCertifySupportEvenWithinStepLimit) {
  for (float height : {.05F, -.05F, .25F, -.25F, .6F, -.6F}) {
    Terrain t;
    t.at(7, 7) = height;
    EXPECT_EQ(Evaluate(t).state, IntrinsicCellState::kUnknown);
  }
}

TEST(LeggedTerrainBasic, MissingLocalSupportAndDegenerateWindowStayUnknown) {
  for (GridIndex p : {GridIndex{7, 7}, GridIndex{8, 7}}) {
    Terrain t;
    t.at(p.x, p.y) = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(Evaluate(t).state, IntrinsicCellState::kUnknown);
  }
  Terrain t;
  for (auto& z : t.z) z = std::numeric_limits<float>::quiet_NaN();
  for (int x = 5; x <= 9; ++x) t.at(x, 7) = 0.F;
  EXPECT_EQ(Evaluate(t).state, IntrinsicCellState::kUnknown);
}

TEST(LeggedTerrainBasic, DerivedMapAndPlannerCrossSupportedStepButRejectCliffAndGap) {
  for (double resolution : {.1, .2})
    for (int scene : {0, 1, 2}) {
      Terrain t;
      t.geometry.resolution_m = resolution;
      t.geometry.width = t.geometry.height = 31;
      t.z.assign(31 * 31, 0.F);
      for (int y = 0; y < 31; ++y)
        for (int x = 16; x < 31; ++x)
          t.at(x, y) = scene == 0 ? .4F : (scene == 1 ? .6F : 0.F);
      if (scene == 2)
        for (int y = 0; y < 31; ++y)
          for (int x = 14; x <= 17; ++x)
            t.at(x, y) = std::numeric_limits<float>::quiet_NaN();
      auto capability = Capability();
      capability.motion_primitives = {{.primitive_id = "forward",
          .kind = LeggedPrimitiveKind::kForward,
          .body_frame_displacement_m = {.x = .2}}};
      const TraversabilityProfile profile{
          .planar_envelope_xy_m = {{-.34, -.165}, {.34, -.165},
                                   {.34, .165}, {-.34, .165}}};
      const auto fine = FineTraversabilityBuilder().Derive(t.snapshot(), capability, profile);
      const Pose2 start{.position_m = {8.5 * resolution, 15.5 * resolution}};
      const RequestLocalPlanningView view(fine, start, 0., {});
      const LocalTarget target{.center = {22.5 * resolution, 15.5 * resolution},
                               .is_final_goal = true};
      const auto result = LeggedLocalPlanner(capability).Plan(
          view, start, target, SteadyClock::time_point::max(), {});
      if (scene == 0) EXPECT_EQ(result.status, LocalPlanResult::Status::kPlanFound);
      else EXPECT_EQ(result.status, LocalPlanResult::Status::kNoPath);
    }
}
}  // namespace
}  // namespace lunar::incremental_navigation
