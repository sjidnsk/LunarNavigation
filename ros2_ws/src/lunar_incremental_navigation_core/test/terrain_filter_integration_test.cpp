#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "lunar_incremental_navigation_core/elevation_map.hpp"
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/types/platform_capability.hpp"
#include "lunar_incremental_navigation_core/types/terrain_filter_method.hpp"
#include "map/terrain_observation_estimator.hpp"

namespace lunar::incremental_navigation {
namespace {

using map::TerrainObservationEstimator;
using map::TerrainObservationParameters;

constexpr std::array kMethods{
    TerrainFitMethod::kBaseline, TerrainFitMethod::kWeightedPlane,
    TerrainFitMethod::kMovingLeastSquares, TerrainFitMethod::kGroupedRansac,
    TerrainFitMethod::kGaussianProcess};
constexpr std::array kFilters{TerrainFilterMethod::kNone,
                              TerrainFilterMethod::kMedian,
                              TerrainFilterMethod::kBilateral};
constexpr GridIndex kCenter{20, 20};

struct FilterFixture {
  GridGeometry geometry{.frame_id = "map", .width = 41, .height = 41,
                        .resolution_m = .05};
  std::vector<float> heights = std::vector<float>(41 * 41, 0.F);
  PersistentElevationMap map;

  float& At(const int x, const int y) { return heights[y * 41 + x]; }
  void Missing(const int x, const int y) {
    At(x, y) = std::numeric_limits<float>::quiet_NaN();
  }
  std::shared_ptr<const ElevationSnapshot> Snapshot() {
    const auto result = map.Apply(
        {.geometry = geometry,
         .elevation_m = heights,
         .map_from_source = {.parent_frame = "map", .child_frame = "map"}});
    if (result.status != ElevationUpdateResult::Status::kApplied)
      throw std::runtime_error("filter test fixture was rejected");
    return map.Snapshot();
  }
};

class CenterIntervalView final : public ElevationRangeView {
 public:
  CenterIntervalView(const ElevationRangeView& raw, const ElevationRange range)
      : raw_(raw), center_range_(range) {}
  const SparseGridGeometry& geometry() const noexcept override {
    return raw_.geometry();
  }
  std::optional<ElevationRange> ElevationRangeAt(const GridIndex cell) const noexcept override {
    return cell == kCenter ? std::optional<ElevationRange>(center_range_)
                           : raw_.ElevationRangeAt(cell);
  }
 private:
  const ElevationRangeView& raw_;
  ElevationRange center_range_;
};

TerrainObservationParameters Parameters(const TerrainFitMethod method,
                                        const TerrainFilterMethod filter) {
  return {.fit_method = method, .filter_method = filter};
}

void ExpectSameEstimate(const std::optional<map::GroundEstimate>& first,
                        const std::optional<map::GroundEstimate>& second) {
  ASSERT_EQ(first.has_value(), second.has_value());
  if (!first) return;
  EXPECT_DOUBLE_EQ(first->surface_height_m, second->surface_height_m);
  EXPECT_DOUBLE_EQ(first->gradient.x, second->gradient.x);
  EXPECT_DOUBLE_EQ(first->gradient.y, second->gradient.y);
  EXPECT_DOUBLE_EQ(first->residual_rms_m, second->residual_rms_m);
  EXPECT_DOUBLE_EQ(first->residual_min_m, second->residual_min_m);
  EXPECT_DOUBLE_EQ(first->residual_max_m, second->residual_max_m);
  EXPECT_DOUBLE_EQ(first->confidence, second->confidence);
  EXPECT_DOUBLE_EQ(first->predictive_variance_m2,
                   second->predictive_variance_m2);
  EXPECT_EQ(first->used_fallback, second->used_fallback);
}

TEST(TerrainFilterIntegration, DefaultNonePreservesEveryMethodAndRawSnapshot) {
  FilterFixture fixture;
  for (int y = 0; y < 41; ++y)
    for (int x = 0; x < 41; ++x)
      fixture.At(x, y) = .2F * (x - 20) * .05F +
                          .009F * std::sin(1.7 * x + .9 * y);
  fixture.Missing(20, 20);
  const auto raw = fixture.Snapshot();
  for (const auto method : kMethods) {
    SCOPED_TRACE(TerrainFitMethodName(method));
    TerrainObservationEstimator implicit(*raw, {.fit_method = method});
    TerrainObservationEstimator explicit_none(
        *raw, Parameters(method, TerrainFilterMethod::kNone));
    for (const auto cell : {kCenter, GridIndex{19, 20}, GridIndex{20, 21}})
      ExpectSameEstimate(implicit.Estimate(cell), explicit_none.Estimate(cell));
  }
  EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
  EXPECT_FLOAT_EQ(raw->ElevationRangeAt({19, 20})->min_m, fixture.At(19, 20));
}

TEST(TerrainFilterIntegration, SubTenCentimeterSpikeCannotBecomeGround) {
  // The 10 cm extrema limit alone would allow this return. Median filtering
  // erases its fit height, so acceptance must still consult raw local evidence.
  for (const auto method : kMethods)
    for (const auto filter : kFilters)
      for (const float height : {-.08F, .08F}) {
        SCOPED_TRACE(TerrainFitMethodName(method));
        SCOPED_TRACE(TerrainFilterMethodName(filter));
        SCOPED_TRACE(height);
        FilterFixture fixture;
        fixture.At(20, 20) = height;
        const auto raw = fixture.Snapshot();
        TerrainObservationEstimator estimator(*raw, Parameters(method, filter));
        EXPECT_FALSE(estimator.Estimate(kCenter));
        ASSERT_TRUE(raw->ElevationRangeAt(kCenter));
        EXPECT_FLOAT_EQ(raw->ElevationRangeAt(kCenter)->min_m, height);
        EXPECT_FLOAT_EQ(raw->ElevationRangeAt(kCenter)->max_m, height);
      }
}

TEST(TerrainFilterIntegration, RawSecondDifferenceSurvivesSmoothing) {
  // Adjacent jumps are only 5 cm (< 6 cm), and every raw residual is < 3 cm.
  // The unsmoothed three-point second difference is 10 cm (> 8 cm).
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      FilterFixture fixture;
      fixture.At(19, 20) = -.025F;
      fixture.At(20, 20) = .025F;
      fixture.At(21, 20) = -.025F;
      const auto raw = fixture.Snapshot();
      TerrainObservationEstimator estimator(*raw, Parameters(method, filter));
      EXPECT_FALSE(estimator.Estimate(kCenter));
    }
}

TEST(TerrainFilterIntegration, ObservedDitchBottomBesideMissingCenterIsNotErased) {
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      FilterFixture fixture;
      for (int y = 0; y < 41; ++y) fixture.At(20, y) = -.08F;
      fixture.Missing(20, 20);
      const auto raw = fixture.Snapshot();
      TerrainObservationEstimator estimator(*raw, Parameters(method, filter));
      EXPECT_FALSE(estimator.Estimate(kCenter));
      EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
      EXPECT_FLOAT_EQ(raw->ElevationRangeAt({20, 19})->min_m, -.08F);
    }
}

TEST(TerrainFilterIntegration, MissingCenterAcrossObservedStepIsNotAFakeRamp) {
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      FilterFixture fixture;
      for (int y = 0; y < 41; ++y)
        for (int x = 21; x < 41; ++x) fixture.At(x, y) = .1F;
      for (int y = 19; y <= 21; ++y)
        for (int x = 19; x <= 21; ++x) fixture.Missing(x, y);
      const auto raw = fixture.Snapshot();
      TerrainObservationEstimator estimator(*raw, Parameters(method, filter));
      EXPECT_FALSE(estimator.Estimate(kCenter));
    }
}

TEST(TerrainFilterIntegration, RawIntervalsRemainIndependentOfFilteredMidpoints) {
  FilterFixture fixture;
  const auto snapshot = fixture.Snapshot();
  // A range view models multiple returns in one cell. Apply replaces the
  // prior frame and does not accumulate a min/max range across two frames.
  const CenterIntervalView raw(*snapshot, {.min_m = -.04F, .max_m = .04F});
  ASSERT_FLOAT_EQ(raw.ElevationRangeAt(kCenter)->min_m, -.04F);
  ASSERT_FLOAT_EQ(raw.ElevationRangeAt(kCenter)->max_m, .04F);
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      TerrainObservationEstimator estimator(raw, Parameters(method, filter));
      EXPECT_FALSE(estimator.Estimate(kCenter));
    }
  EXPECT_FLOAT_EQ(snapshot->ElevationRangeAt(kCenter)->max_m, 0.F);
  EXPECT_FLOAT_EQ(raw.ElevationRangeAt(kCenter)->min_m, -.04F);
  EXPECT_FLOAT_EQ(raw.ElevationRangeAt(kCenter)->max_m, .04F);
}

TEST(TerrainFilterIntegration, SmoothedOutliersCannotSupplyRawSpatialRank) {
  // Two off-axis outliers acquire zero fit height under median filtering.
  // Removing them from raw consensus leaves a line, not a supported surface.
  FilterFixture fixture;
  std::fill(fixture.heights.begin(), fixture.heights.end(),
            std::numeric_limits<float>::quiet_NaN());
  for (int x = 14; x <= 26; ++x) fixture.At(x, 20) = 0.F;
  fixture.At(20, 19) = fixture.At(20, 21) = .08F;
  const auto raw = fixture.Snapshot();
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      TerrainObservationEstimator estimator(*raw, Parameters(method, filter));
      EXPECT_FALSE(estimator.Estimate(kCenter));
    }
}

TEST(TerrainFilterIntegration, RansacRejectsAFilterCreatedBridgeBetweenRawGroups) {
  // At 5 cm resolution RANSAC's spatial link is 16 cm. The two original
  // zero-height banks are 20 cm apart and each has only three supporting
  // points. Median erases the isolated 8 cm bridge return and joins both
  // banks in fit space; raw consensus must still preserve separate groups.
  FilterFixture fixture;
  std::fill(fixture.heights.begin(), fixture.heights.end(),
            std::numeric_limits<float>::quiet_NaN());
  for (const int x : {18, 22})
    for (const int y : {18, 20, 22}) fixture.At(x, y) = 0.F;
  fixture.At(20, 22) = .08F;
  const auto raw = fixture.Snapshot();
  for (const auto filter : kFilters) {
    SCOPED_TRACE(TerrainFilterMethodName(filter));
    TerrainObservationEstimator estimator(
        *raw, Parameters(TerrainFitMethod::kGroupedRansac, filter));
    EXPECT_FALSE(estimator.Estimate(kCenter));
    EXPECT_FLOAT_EQ(raw->ElevationRangeAt({20, 22})->min_m, .08F);
    EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
  }
}

TEST(TerrainFilterIntegration, RansacFinalRawMaskCannotLoseItsOnlySpatialBridge) {
  // Raw and median-fit group masks both include seven connected points. Raw
  // residual rechecking removes the 3.5 cm bridge from the median plane, so
  // their final intersection must not retain two disconnected three-point banks.
  for(const float height:{-.035F,.035F}) {
    SCOPED_TRACE(height);
    FilterFixture fixture;
    std::fill(fixture.heights.begin(),fixture.heights.end(),
              std::numeric_limits<float>::quiet_NaN());
    for(const int x:{18,22})
      for(const int y:{18,20,22}) fixture.At(x,y)=0.F;
    fixture.At(20,22)=height;
    const auto raw=fixture.Snapshot();
    TerrainObservationEstimator original(*raw,
        Parameters(TerrainFitMethod::kGroupedRansac,TerrainFilterMethod::kNone));
    ASSERT_TRUE(original.Estimate(kCenter));  // Existing none behaviour is retained.
    TerrainObservationEstimator median(*raw,
        Parameters(TerrainFitMethod::kGroupedRansac,TerrainFilterMethod::kMedian));
    EXPECT_FALSE(median.Estimate(kCenter));
    EXPECT_FLOAT_EQ(raw->ElevationRangeAt({20,22})->min_m,height);
    EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
  }
}

TEST(TerrainFilterIntegration, RawRmsAndConfidenceAreNotTheFilteredFitError) {
  FilterFixture fixture;
  // Nine isolated 2.5 cm returns in the preferred 9x9 candidate. All are raw
  // inliers. Median sends their fit heights to zero, not their evidence error.
  for (int y = 0; y < 41; ++y)
    for (int x = 0; x < 41; ++x)
      if ((x - 20 + 5) % 3 == 0 && (y - 20 + 5) % 3 == 0)
        fixture.At(x, y) = .025F;
  const auto raw = fixture.Snapshot();
  for (const auto method : kMethods) {
    SCOPED_TRACE(TerrainFitMethodName(method));
    TerrainObservationEstimator estimator(
        *raw, Parameters(method, TerrainFilterMethod::kMedian));
    const auto ground = estimator.Estimate(kCenter);
    ASSERT_TRUE(ground);
    EXPECT_NEAR(ground->surface_height_m, 0., 1e-8);
    EXPECT_NEAR(ground->residual_rms_m, .025 / 3., 1e-8);
    EXPECT_NEAR(ground->residual_min_m, 0., 1e-8);
    EXPECT_NEAR(ground->residual_max_m, .025, 1e-8);
    // GP may additionally reduce confidence for predictive uncertainty.
    EXPECT_LE(ground->confidence, 1. / (1. + (.025 / 3.) / .02) + 1e-8);
    EXPECT_GT(ground->confidence, .2);
  }
}

TEST(TerrainFilterIntegration, FilteringCannotFillBeyondNearestEvidenceLimit) {
  FilterFixture fixture;
  for (int y = 0; y < 41; ++y)
    for (int x = 16; x <= 24; ++x) fixture.Missing(x, y);
  const auto raw = fixture.Snapshot();
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      TerrainObservationEstimator estimator(*raw, Parameters(method, filter));
      EXPECT_FALSE(estimator.Estimate(kCenter));
      EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
    }
}

TEST(TerrainFilterIntegration, QueryOrderDoesNotCreateNewEvidenceOrChangeScores) {
  FilterFixture fixture;
  for (int y = 0; y < 41; ++y)
    for (int x = 0; x < 41; ++x)
      fixture.At(x, y) = .2F * (x - 20) * .05F;
  for (int y = 19; y <= 21; ++y)
    for (int x = 19; x <= 21; ++x) fixture.Missing(x, y);
  const auto raw = fixture.Snapshot();
  for (const auto method : kMethods)
    for (const auto filter : kFilters) {
      SCOPED_TRACE(TerrainFitMethodName(method));
      SCOPED_TRACE(TerrainFilterMethodName(filter));
      TerrainObservationEstimator direct(*raw, Parameters(method, filter));
      TerrainObservationEstimator reordered(*raw, Parameters(method, filter));
      const auto expected = direct.Estimate(kCenter);
      for (const auto cell : {GridIndex{19, 20}, GridIndex{20, 19},
                              GridIndex{21, 20}, GridIndex{20, 21}})
        (void)reordered.Estimate(cell);
      ExpectSameEstimate(expected, reordered.Estimate(kCenter));
      const auto evaluations = reordered.evaluated_cells();
      ExpectSameEstimate(expected, reordered.Estimate(kCenter));
      EXPECT_EQ(reordered.evaluated_cells(), evaluations);
      EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
      EXPECT_FALSE(raw->ElevationRangeAt({19, 20}));
    }
}

LeggedCapability Capability() {
  LeggedCapability capability;
  capability.maximum_slope_rad = .52;
  capability.maximum_step_height_m = .5;
  capability.maximum_gap_width_m = .2;
  capability.unknown_is_traversable = false;
  return capability;
}

TraversabilityProfile Profile() {
  TraversabilityProfile profile;
  profile.planar_envelope_xy_m = {{-.05, -.05}, {.05, -.05},
                                 {.05, .05}, {-.05, .05}};
  profile.terrain_background_half_width_m = .2;
  profile.preferred_clearance_m = .02;
  profile.confidence_weight = 2.;
  return profile;
}

TEST(TerrainFilterIntegration, InvalidFilterEnumIsRejectedAtPublicBoundaries) {
  FilterFixture fixture;
  const auto raw = fixture.Snapshot();
  const auto invalid = static_cast<TerrainFilterMethod>(255);
  EXPECT_THROW(TerrainObservationEstimator(*raw, {.filter_method = invalid}),
               std::invalid_argument);
  const auto capability = Capability();
  auto profile = Profile();
  profile.terrain_filter_method = invalid;
  EXPECT_THROW(FineCellEvaluator(*raw, capability, profile), std::invalid_argument);
  EXPECT_THROW(FineTraversabilityBuilder().Derive(raw, capability, profile),
               std::invalid_argument);
  EXPECT_THROW(ParseTerrainFilterMethod("smooth_everything"), std::invalid_argument);
}

void ExpectSameFine(const FineTraversabilitySnapshot& first,
                    const FineTraversabilitySnapshot& second) {
  for (int y = 14; y <= 26; ++y)
    for (int x = 14; x <= 26; ++x) {
      const GridIndex cell{x, y};
      SCOPED_TRACE(x);
      SCOPED_TRACE(y);
      EXPECT_EQ(first.State(cell), second.State(cell));
      EXPECT_DOUBLE_EQ(first.Confidence(cell), second.Confidence(cell));
      EXPECT_DOUBLE_EQ(first.TraversalCost(cell), second.TraversalCost(cell));
      const auto a = first.PlanningElevationRangeAt(cell);
      const auto b = second.PlanningElevationRangeAt(cell);
      ASSERT_EQ(a.has_value(), b.has_value());
      if (a) {
        EXPECT_FLOAT_EQ(a->min_m, b->min_m);
        EXPECT_FLOAT_EQ(a->max_m, b->max_m);
      }
    }
}

TEST(TerrainFilterIntegration, ProfileFilterSwitchInvalidatesCachedFineDerivation) {
  FilterFixture fixture;
  for (int y = 0; y < 41; ++y)
    for (int x = 0; x < 41; ++x)
      if ((x - 20 + 5) % 3 == 0 && (y - 20 + 5) % 3 == 0)
        fixture.At(x, y) = .025F;
  fixture.Missing(20, 20);
  const auto raw = fixture.Snapshot();
  const auto capability = Capability();
  auto profile = Profile();
  FineTraversabilityBuilder builder;
  const auto baseline = builder.Derive(raw, capability, profile);
  EXPECT_EQ(builder.Derive(raw, capability, profile, baseline), baseline);
  std::array<std::string, 3> hashes;
  hashes[0] = baseline->platform_profile_hash();
  auto previous = baseline;
  for (std::size_t i = 1; i < kFilters.size(); ++i) {
    profile.terrain_filter_method = kFilters[i];
    const auto switched = builder.Derive(raw, capability, profile, previous);
    const auto fresh = builder.Derive(raw, capability, profile);
    ASSERT_NE(switched, previous);
    hashes[i] = switched->platform_profile_hash();
    EXPECT_NE(hashes[i], hashes[0]);
    ExpectSameFine(*switched, *fresh);
    EXPECT_EQ(builder.Derive(raw, capability, profile, switched), switched);
    previous = switched;
  }
  EXPECT_NE(hashes[1], hashes[2]);
  profile.terrain_filter_method = TerrainFilterMethod::kNone;
  const auto restored = builder.Derive(raw, capability, profile, previous);
  EXPECT_EQ(restored->platform_profile_hash(), hashes[0]);
  ExpectSameFine(*restored, *baseline);
  EXPECT_FALSE(raw->ElevationRangeAt(kCenter));
}

TEST(TerrainFilterIntegration, FineCellEvaluatorForwardsFilterToGroundEstimator) {
  FilterFixture fixture;
  for (int y = 0; y < 41; ++y)
    for (int x = 0; x < 41; ++x)
      if ((x - 20 + 5) % 3 == 0 && (y - 20 + 5) % 3 == 0)
        fixture.At(x, y) = .025F;
  const auto raw = fixture.Snapshot();
  const auto capability = Capability();
  auto profile = Profile();
  FineCellEvaluator baseline(*raw, capability, profile);
  profile.terrain_filter_method = TerrainFilterMethod::kMedian;
  FineCellEvaluator median(*raw, capability, profile);
  const auto base_height = baseline.GroundHeight(kCenter);
  const auto median_height = median.GroundHeight(kCenter);
  ASSERT_TRUE(base_height);
  ASSERT_TRUE(median_height);
  EXPECT_GT(*base_height, .001F);
  EXPECT_NEAR(*median_height, 0.F, 1e-8);
}

TEST(TerrainFilterIntegration, FilteredIncrementalUpdateMatchesFreshRawEvidence) {
  for (const auto filter : {TerrainFilterMethod::kMedian,
                            TerrainFilterMethod::kBilateral}) {
    SCOPED_TRACE(TerrainFilterMethodName(filter));
    FilterFixture fixture;
    fixture.Missing(20, 20);
    const auto old_raw = fixture.Snapshot();
    auto profile = Profile();
    profile.terrain_filter_method = filter;
    const auto capability = Capability();
    FineTraversabilityBuilder builder;
    const auto previous = builder.Derive(old_raw, capability, profile);
    ASSERT_EQ(previous->State({25, 20}), FineCellState::kFree);
    // The changed sample is outside the preferred center window, but inside
    // the adaptive dependency. It must invalidate nearby filtered caches too.
    fixture.At(25, 20) = .7F;
    const auto updated = fixture.Snapshot();
    const auto incremental = builder.Derive(updated, capability, profile, previous);
    const auto fresh = builder.Derive(updated, capability, profile);
    EXPECT_EQ(incremental->platform_profile_hash(), previous->platform_profile_hash());
    ExpectSameFine(*incremental, *fresh);
    // Both BLOCKED and UNKNOWN refuse passage. A spike can remove continuous
    // ground support before platform relief classification is available.
    auto unfiltered_profile = profile;
    unfiltered_profile.terrain_filter_method = TerrainFilterMethod::kNone;
    const auto unfiltered = builder.Derive(updated, capability, unfiltered_profile);
    EXPECT_NE(unfiltered->State({25, 20}), FineCellState::kFree);
    EXPECT_NE(fresh->State({25, 20}), FineCellState::kFree);
    EXPECT_EQ(previous->State({25, 20}), FineCellState::kFree);
    EXPECT_FLOAT_EQ(old_raw->ElevationRangeAt({25, 20})->max_m, 0.F);
    EXPECT_FLOAT_EQ(updated->ElevationRangeAt({25, 20})->max_m, .7F);
    EXPECT_FALSE(updated->ElevationRangeAt(kCenter));
  }
}

}  // namespace
}  // namespace lunar::incremental_navigation
