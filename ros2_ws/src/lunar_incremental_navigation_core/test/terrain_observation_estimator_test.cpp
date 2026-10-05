#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "map/terrain_observation_estimator.hpp"
using namespace lunar::incremental_navigation;
using namespace lunar::incremental_navigation::map;
namespace {
struct GroundFixture {
  GridGeometry geometry{.frame_id="map", .width=41, .height=41, .resolution_m=.05};
  std::vector<float> heights=std::vector<float>(41*41,std::numeric_limits<float>::quiet_NaN());
  PersistentElevationMap map;
  auto Snapshot() {
    (void)map.Apply({.geometry=geometry,.elevation_m=heights,
                    .map_from_source={.parent_frame="map",.child_frame="map"}});
    return map.Snapshot();
  }
  void Plane(int stride=4, double gradient=.4) {
    for(int y=0;y<41;y+=stride) for(int x=0;x<41;x+=stride)
      heights[y*41+x]=gradient*x*.05;
  }
};
TEST(TerrainObservationEstimator, DeterministicAcrossOrderAndNeverWritesInferredEvidence) {
  GroundFixture f; f.Plane(); auto raw=f.Snapshot();
  TerrainObservationEstimator a(*raw),b(*raw);
  auto expected=a.Estimate({18,18}); ASSERT_TRUE(expected);
  (void)b.Estimate({19,19}); auto actual=b.Estimate({18,18}); ASSERT_TRUE(actual);
  EXPECT_DOUBLE_EQ(expected->surface_height_m,actual->surface_height_m);
  EXPECT_DOUBLE_EQ(expected->confidence,actual->confidence);
  EXPECT_NEAR(expected->gradient.x,.4,1e-6);
  EXPECT_NEAR(expected->gradient.y,0.,1e-6);
  EXPECT_NEAR(expected->surface_height_m,.36,1e-6);
  EXPECT_FALSE(raw->ElevationRangeAt({18,18}));
  auto* stable=&a.Estimate({18,18});
  for(int y=0;y<41;++y)for(int x=0;x<41;++x)(void)a.Estimate({x,y});
  EXPECT_EQ(stable,&a.Estimate({18,18})); EXPECT_EQ(a.evaluated_cells(),1681u);
  for(int y=0;y<41;++y)for(int x=0;x<41;++x)(void)a.Estimate({x,y});
  EXPECT_EQ(a.evaluated_cells(),1681u); EXPECT_GT(a.allocated_bytes(),0u);
}
TEST(TerrainObservationEstimator, FailureCachedAndInputViewsIsolated) {
  GroundFixture f; auto empty=f.Snapshot(); TerrainObservationEstimator a(*empty);
  EXPECT_FALSE(a.Estimate({18,18}));EXPECT_FALSE(a.Estimate({18,18}));
  EXPECT_EQ(a.evaluated_cells(),1u);
  f.Plane(); auto filled=f.Snapshot();TerrainObservationEstimator b(*filled);
  EXPECT_TRUE(b.Estimate({18,18}));EXPECT_FALSE(a.Estimate({18,18}));
  EXPECT_FALSE(a.Estimate({-1,-1}));EXPECT_EQ(a.evaluated_cells(),1u);
}
TEST(TerrainObservationEstimator, DenseAndSparseSteepPlanesHaveSameGeometry) {
  GroundFixture f,g;f.Plane(1,2.);g.Plane(4,2.);
  auto dense=f.Snapshot(),sparse=g.Snapshot();
  TerrainObservationEstimator a(*dense),b(*sparse);
  const auto da=a.Estimate({18,18}),sp=b.Estimate({18,18});ASSERT_TRUE(da);ASSERT_TRUE(sp);
  EXPECT_NEAR(da->gradient.x,sp->gradient.x,1e-5);
  EXPECT_NEAR(da->surface_height_m,sp->surface_height_m,1e-5);
  EXPECT_GT(da->confidence,sp->confidence);
}
TEST(TerrainObservationEstimator, DegenerateAndConflictingEvidenceRejected) {
  for(int scene=0;scene<4;++scene) {
    GroundFixture f;
    for(int y=16;y<=24;++y)for(int x=16;x<=24;++x) {
      if(scene==0 && x<21)continue;
      if(scene==1 && x!=y)continue;
      if(scene==2 && (x+y)%2)f.heights[y*41+x]=1.f;
      else f.heights[y*41+x]=scene==3?std::numeric_limits<float>::infinity():0.f;
    }
    auto raw=f.Snapshot();TerrainObservationEstimator a(*raw);
    EXPECT_FALSE(a.Estimate({20,20})) << scene;
  }
}
TEST(TerrainObservationEstimator, RawIntervalsAndIsolatedSpikesAreNotSmoothedAway) {
  GroundFixture f;f.Plane(1,0.);auto before=f.Snapshot();
  f.heights[20*41+20]=.4f;auto raw=f.Snapshot();TerrainObservationEstimator a(*raw);
  EXPECT_FALSE(a.Estimate({20,20}));
  EXPECT_FLOAT_EQ(raw->ElevationRangeAt({20,20})->max_m,.4f);
  EXPECT_FLOAT_EQ(before->ElevationRangeAt({20,20})->max_m,0.f);
}
TEST(TerrainObservationEstimator, InvalidParametersRejectedAndCoarseHolesPreserved) {
  GroundFixture f;f.Plane(1,0.);auto raw=f.Snapshot();
  EXPECT_THROW(TerrainObservationEstimator(*raw,{.support_axis_distance_m=0.}),std::invalid_argument);
  EXPECT_THROW(TerrainObservationEstimator(*raw,{.maximum_plane_residual_m=-.1}),std::invalid_argument);
  EXPECT_THROW(TerrainObservationEstimator(*raw,{.minimum_confidence=1.1}),std::invalid_argument);
  f.geometry.resolution_m=.2;f.heights[20*41+20]=std::numeric_limits<float>::quiet_NaN();
  PersistentElevationMap other;
  (void)other.Apply({.geometry=f.geometry,.elevation_m=f.heights,.map_from_source={.parent_frame="map",.child_frame="map"}});
  auto coarse=other.Snapshot();TerrainObservationEstimator b(*coarse);
  EXPECT_FALSE(b.Estimate({20,20}));
}
}

namespace {
class AssumedGroundView final : public ElevationRangeView {
 public:
  explicit AssumedGroundView(const ElevationRangeView& raw) : raw_(raw) {}
  const SparseGridGeometry& geometry() const noexcept override { return raw_.geometry(); }
  std::optional<ElevationRange> ElevationRangeAt(GridIndex cell) const noexcept override {
    if (!geometry().Contains(cell)) return {};
    return raw_.ElevationRangeAt(cell).value_or(ElevationRange{.min_m=0.f,.max_m=0.f});
  }
 private:
  const ElevationRangeView& raw_;
};
TEST(TerrainObservationEstimator, AssumedViewCannotPopulateRawCache) {
  GroundFixture f; auto raw=f.Snapshot(); AssumedGroundView assumed(*raw);
  TerrainObservationEstimator original(*raw), temporary(assumed);
  EXPECT_FALSE(original.Estimate({20,20}));
  ASSERT_TRUE(temporary.Estimate({20,20}));
  EXPECT_FALSE(original.Estimate({20,20}));
  EXPECT_FALSE(raw->ElevationRangeAt({20,20}));
  EXPECT_EQ(original.evaluated_cells(),1u);
}
}

TEST(TerrainObservationEstimator, NoisyObservedGroundUsesEstimatedSlopeAndKeepsRawExtrema) {
  GroundFixture f;
  for(int y=0;y<41;++y)for(int x=0;x<41;++x)
    f.heights[y*41+x]=.035f*std::sin(1.7*x)+.008f*std::cos(1.3*y);
  auto raw=f.Snapshot();TerrainObservationEstimator estimator(*raw);
  auto ground=estimator.Estimate({20,20});ASSERT_TRUE(ground);
  auto patch=MeasureTerrainPatch(*raw,{20,20},.2,estimator);ASSERT_TRUE(patch);
  EXPECT_NEAR(patch->median_slope_rad,std::atan(std::hypot(ground->gradient.x,ground->gradient.y)),1e-9);
  EXPECT_NEAR(patch->refined_slope_rad,patch->median_slope_rad,1e-9);
  EXPECT_LT(patch->median_slope_rad,.2);
  EXPECT_GT(patch->confidence,0.2);EXPECT_LT(patch->confidence,1.);
  EXPECT_GT(patch->high-patch->low,.02);
}
TEST(TerrainObservationEstimator, AdaptiveSupportExtendsDependencyWithoutRecursiveFilling) {
  GroundFixture f;
  for(int y: {15,20,25})for(int x: {15,20,25})
    if(x!=20 || y!=20)f.heights[y*41+x]=0.;
  // Four axial samples are 25 cm away: nearest-evidence bound rejects.
  auto raw=f.Snapshot();TerrainObservationEstimator a(*raw);
  EXPECT_GE(a.dependency_radius_cells(),6);
  EXPECT_FALSE(a.Estimate({20,20}));
  f.heights[20*41+19]=0.;auto more=f.Snapshot();TerrainObservationEstimator b(*more);
  EXPECT_TRUE(b.Estimate({20,20}));
  EXPECT_FALSE(more->ElevationRangeAt({20,20}));
}

#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
TEST(TerrainObservationEstimator, SupportedCenterDoesNotRequireNeighborsToRepeatItsEvidence) {
  GroundFixture f;
  for(GridIndex p : {GridIndex{19,20},{20,19},{21,20},{20,21}})
    f.heights[p.y*41+p.x]=0.;
  auto raw=f.Snapshot();TerrainObservationEstimator obs(*raw);
  auto patch=MeasureTerrainPatch(*raw,{20,20},.2,obs);ASSERT_TRUE(patch);
  ASSERT_TRUE(patch->continuous_ground);ASSERT_FALSE(patch->central_complete);
  LeggedCapability capability;capability.maximum_slope_rad=.52;capability.maximum_step_height_m=.5;
  EXPECT_EQ(MakePlatformElevationEvaluator(capability,.2)->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kFree);
  // A center estimate does not certify the robot's entire footprint.
  TraversabilityProfile profile;profile.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}};
  EXPECT_NE(FineCellEvaluator(*raw,capability,profile).Evaluate({20,20}).state,FineCellState::kFree);
}
TEST(TerrainObservationEstimator, SmallStepsAndThinObstaclesCannotBecomeInferredGround) {
  for(double resolution:{.025,.05,.1})for(float height:{-.1f,.1f,.32f,.7f}) {
    GroundFixture f;f.geometry.resolution_m=resolution;f.Plane(1,0.);
    for(int y=0;y<41;++y)for(int x=21;x<41;++x)f.heights[y*41+x]=height;
    f.heights[20*41+20]=std::numeric_limits<float>::quiet_NaN();
    auto raw=f.Snapshot();TerrainObservationEstimator obs(*raw);
    EXPECT_FALSE(obs.Estimate({20,20}))<<resolution<<" "<<height;
  }
}

TEST(TerrainObservationEstimator, ThinObservedWallBesideHolesRemainsAnObstacle) {
  GroundFixture f;f.Plane(1,0.);
  for(int y=0;y<41;++y){f.heights[y*41+20]=.6f;f.heights[y*41+21]=std::numeric_limits<float>::quiet_NaN();}
  auto raw=f.Snapshot();WheeledCapability wheel;wheel.maximum_slope_rad=.35;
  wheel.maximum_local_obstacle_relief_m=.2;wheel.minimum_underbody_clearance_m=.21;
  EXPECT_EQ(MakePlatformElevationEvaluator(wheel,.4)->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kBlocked);
  EXPECT_FLOAT_EQ(raw->ElevationRangeAt({20,20})->max_m,.6f);
}
TEST(TerrainObservationEstimator, OuterAdaptiveSupportUpdateMatchesFullDerivation) {
  GroundFixture f;
  for(int y:{15,20,25})for(int x:{15,20,25})
    if(x!=20 || y!=20)f.heights[y*41+x]=0.;
  f.heights[20*41+19]=0.;
  auto raw=f.Snapshot();WheeledCapability wheel;wheel.maximum_slope_rad=.35;
  wheel.maximum_local_obstacle_relief_m=.2;wheel.minimum_underbody_clearance_m=.21;
  TraversabilityProfile profile;profile.planar_envelope_xy_m={{-.001,-.001},{.001,-.001},{.001,.001},{-.001,.001}};profile.terrain_background_half_width_m=.2;profile.confidence_weight=2.;
  FineTraversabilityBuilder builder;auto before=builder.Derive(raw,wheel,profile);
  TerrainObservationEstimator a(*raw);ASSERT_TRUE(a.Estimate({20,20}));
  f.heights[20*41+25]=.5f;auto updated=f.Snapshot();
  TerrainObservationEstimator b(*updated);EXPECT_FALSE(b.Estimate({20,20}));
  auto incremental=builder.Derive(updated,wheel,profile,before);
  auto full=builder.Derive(updated,wheel,profile);
  for(int y=0;y<41;++y)for(int x=0;x<41;++x) {
    EXPECT_EQ(incremental->State({x,y}),full->State({x,y}));
    EXPECT_EQ(incremental->PlanningElevationRangeAt({x,y}),full->PlanningElevationRangeAt({x,y}));
    EXPECT_DOUBLE_EQ(incremental->Confidence({x,y}),full->Confidence({x,y}));
    EXPECT_DOUBLE_EQ(incremental->TraversalCost({x,y}),full->TraversalCost({x,y}));
  }
}

TEST(TerrainObservationEstimator, GroundHeightOnlyUpdatePublishesNewTileAndKeepsOldSnapshot) {
  GroundFixture f;f.Plane(4,0.);auto raw=f.Snapshot();
  WheeledCapability wheel;wheel.maximum_slope_rad=.35;wheel.maximum_local_obstacle_relief_m=.2;wheel.minimum_underbody_clearance_m=.21;
  TraversabilityProfile p;p.planar_envelope_xy_m={{-.001,-.001},{.001,-.001},{.001,.001},{-.001,.001}};
  p.terrain_background_half_width_m=.2;p.slope_weight=p.relief_weight=p.clearance_weight=p.confidence_weight=0.;
  FineTraversabilityBuilder b;auto old=b.Derive(raw,wheel,p);
  ASSERT_TRUE(old->PlanningElevationRangeAt({18,18}));
  for(auto& z:f.heights)if(std::isfinite(z))z=.02f;
  auto updated=f.Snapshot();auto inc=b.Derive(updated,wheel,p,old),full=b.Derive(updated,wheel,p);
  ASSERT_TRUE(inc->PlanningElevationRangeAt({18,18}));
  EXPECT_EQ(inc->State({18,18}),old->State({18,18}));
  EXPECT_DOUBLE_EQ(inc->TraversalCost({18,18}),old->TraversalCost({18,18}));
  EXPECT_GT(inc->PlanningElevationRangeAt({18,18})->min_m,old->PlanningElevationRangeAt({18,18})->min_m);
  EXPECT_EQ(inc->PlanningElevationRangeAt({18,18}),full->PlanningElevationRangeAt({18,18}));
  EXPECT_FALSE(updated->ElevationRangeAt({18,18}));EXPECT_FLOAT_EQ(old->PlanningElevationRangeAt({18,18})->min_m,0.f);
  EXPECT_FALSE(inc->changed_tiles().empty());
}

TEST(TerrainObservationEstimator, DistantSmallReturnDoesNotVetoSupportedHole) {
  GroundFixture f;f.Plane(1,0.);
  f.heights[20*41+20]=std::numeric_limits<float>::quiet_NaN();
  // Sparse central support forces a larger fit; the anomalous return is 15cm
  // away and must not veto a different, well-supported target cell.
  for(int y=18;y<=22;++y)for(int x=18;x<=22;++x)
    f.heights[y*41+x]=std::numeric_limits<float>::quiet_NaN();
  f.heights[20*41+23]=.09f;
  auto raw=f.Snapshot();TerrainObservationEstimator obs(*raw);
  const auto estimate=obs.Estimate({20,20});ASSERT_TRUE(estimate);
  EXPECT_NEAR(estimate->surface_height_m,0.,.005);
  EXPECT_GT(estimate->confidence,.2);EXPECT_LT(estimate->confidence,1.);
  EXPECT_FALSE(raw->ElevationRangeAt({20,20}));
  EXPECT_FLOAT_EQ(raw->ElevationRangeAt({23,20})->max_m,.09f);
  EXPECT_FALSE(obs.Estimate({23,20}));
}

TEST(TerrainObservationEstimator, NearbyJumpStillRejectsHoleAtMultipleResolutions) {
  for(double resolution:{.025,.05,.1})for(float height:{-.09f,.09f,-.5f,.7f}) {
    GroundFixture f;f.geometry.resolution_m=resolution;f.Plane(1,0.);
    f.heights[20*41+20]=std::numeric_limits<float>::quiet_NaN();
    f.heights[20*41+21]=height;
    auto raw=f.Snapshot();TerrainObservationEstimator obs(*raw);
    EXPECT_FALSE(obs.Estimate({20,20}))<<resolution<<" "<<height;
    EXPECT_FLOAT_EQ(raw->ElevationRangeAt({21,20})->min_m,height);
  }
}

TEST(TerrainObservationEstimator, RemoteSupportedStepAcrossMissingCenterIsNotAFakeRamp) {
  for(int hole_radius:{1,2})for(int direction:{0,1,2})for(float height:{-.1f,.1f}) {
    GroundFixture f;f.Plane(1,0.);
    for(int y=0;y<41;++y)for(int x=0;x<41;++x)
      if(direction==0?x>20:direction==1?y>20:x+y>40)f.heights[y*41+x]=height;
    for(int y=20-hole_radius;y<=20+hole_radius;++y)
      for(int x=20-hole_radius;x<=20+hole_radius;++x)
        f.heights[y*41+x]=std::numeric_limits<float>::quiet_NaN();
    auto raw=f.Snapshot();TerrainObservationEstimator obs(*raw);
    EXPECT_FALSE(obs.Estimate({20,20}))<<hole_radius<<" "<<direction<<" "<<height;
  }
}

TEST(TerrainObservationEstimator, SameSideRemoteReturnsDoNotEraseSupportedGroundElsewhere) {
  GroundFixture f;f.Plane(1,0.);
  for(int y=18;y<=22;++y)for(int x=18;x<=22;++x)
    f.heights[y*41+x]=std::numeric_limits<float>::quiet_NaN();
  f.heights[19*41+23]=f.heights[21*41+23]=.09f;
  auto raw=f.Snapshot();TerrainObservationEstimator obs(*raw);
  auto e=obs.Estimate({20,20});ASSERT_TRUE(e);EXPECT_NEAR(e->surface_height_m,0.,.005);
  EXPECT_FALSE(obs.Estimate({23,19}));EXPECT_FALSE(obs.Estimate({23,21}));
}
