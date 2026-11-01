#include <gtest/gtest.h>
#include "lunar_incremental_navigation_core/wheel_local_planner.hpp"
#include "lunar_incremental_navigation_core/local_planning.hpp"
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <vector>
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
using namespace lunar::incremental_navigation;
namespace {
WheeledCapability Wheel(){WheeledCapability c;c.maximum_slope_rad=.3490658504;c.maximum_local_obstacle_relief_m=.2;c.minimum_underbody_clearance_m=.21;return c;}
struct Fixture {
 GridGeometry g{.frame_id="map",.width=41,.height=41,.resolution_m=.1};
 std::vector<float> z=std::vector<float>(41*41,0.f);PersistentElevationMap map;
 auto snap(){(void)map.Apply({.geometry=g,.elevation_m=z,.map_from_source={.parent_frame="map",.child_frame="map"}});return map.Snapshot();}
};
TEST(TerrainWindow, SupportedLocalPatchDoesNotNeedSixtyPercentOfBackground){
 Fixture f;std::fill(f.z.begin(),f.z.end(),std::numeric_limits<float>::quiet_NaN());
 for(int y=18;y<=22;++y)for(int x=18;x<=22;++x)f.z[y*41+x]=0;
 auto raw=f.snap();
 EXPECT_EQ(MakePlatformElevationEvaluator(Wheel())->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kFree);
 EXPECT_EQ(MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kFree);
}
TEST(TerrainWindow, PlanarSlopesAndStepsAcrossResolutions){
 for(double r:{.2,.1,.05})for(double slope:{0.,.2,.45}){
 Fixture f;f.g.resolution_m=r;
 for(int y=0;y<41;++y)for(int x=0;x<41;++x)f.z[y*41+x]=std::tan(slope)*(x-20)*r;
 auto e=MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*f.snap(),{20,20});
 EXPECT_NEAR(e.slope_rad,slope,1e-6);EXPECT_EQ(e.state,slope>.35?IntrinsicCellState::kBlocked:IntrinsicCellState::kFree);
 }
 for(double r:{.2,.1,.05})for(float h:{.19f,.21f}){
 Fixture f;f.g.resolution_m=r;for(int y=0;y<41;++y)for(int x=21;x<41;++x)f.z[y*41+x]=h;
 auto e=MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*f.snap(),{20,20});
 EXPECT_EQ(e.state,h>.2?IntrinsicCellState::kBlocked:IntrinsicCellState::kFree);
 }
}
TEST(TerrainWindow, InvalidWidthRejected){
 EXPECT_THROW(MakePlatformElevationEvaluator(Wheel(),-.1),std::invalid_argument);
 EXPECT_THROW(MakePlatformElevationEvaluator(Wheel(),std::numeric_limits<double>::infinity()),std::invalid_argument);
}
}

TEST(TerrainWindow, QuantizationAndBounds){
 EXPECT_EQ(TerrainBackgroundRadiusCells(.4,.2),2);
 EXPECT_EQ(TerrainBackgroundRadiusCells(.4,.1),4);
 EXPECT_EQ(TerrainBackgroundRadiusCells(.2,.05),4);
 EXPECT_EQ(TerrainBackgroundRadiusCells(.3,.1),3);
 EXPECT_EQ(TerrainBackgroundRadiusCells(.4,.3),2);
 EXPECT_THROW(static_cast<void>(TerrainBackgroundRadiusCells(.4,.001)),std::invalid_argument);
}
TEST(TerrainWindow, BackgroundDoesNotExpandResidualOrSupportWindow){
 Fixture f;f.z[20*41+20]=.4f; // isolated return must not gain support from distant plateau
 for(int y=16;y<=24;++y)for(int x=16;x<=24;++x)
  if(std::abs(x-20)>2 || std::abs(y-20)>2)f.z[y*41+x]=.4f;
 auto raw=f.snap();
 EXPECT_EQ(MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kUnknown);
 Fixture wall;for(int y=0;y<41;++y)for(int x=23;x<41;++x)wall.z[y*41+x]=2.f;
 EXPECT_EQ(MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*wall.snap(),{20,20}).state,IntrinsicCellState::kFree);
}
TEST(TerrainWindow, IncrementalAndProfileChangesMatchFullDerivation){
 for(double r:{.2,.1,.05}){
 Fixture f;f.g.resolution_m=r;TraversabilityProfile p;
 p.planar_envelope_xy_m={{-.15,-.1},{.15,-.1},{.15,.1},{-.15,.1}};
 p.terrain_background_half_width_m=.4;p.preferred_clearance_m=.4;
 p.clearance_weight=2;p.slope_weight=1;p.relief_weight=1;
 FineTraversabilityBuilder b;auto previous=b.Derive(f.snap(),Wheel(),p);
 for(int epoch=0;epoch<3;++epoch){
  for(int y=17;y<=23;++y)for(int x=23;x<=25;++x)f.z[y*41+x]=float(.12*(epoch+1));
  auto raw=f.snap();auto inc=b.Derive(raw,Wheel(),p,previous),full=b.Derive(raw,Wheel(),p);
  for(int y=0;y<41;++y)for(int x=0;x<41;++x){EXPECT_EQ(inc->State({x,y}),full->State({x,y}));EXPECT_DOUBLE_EQ(inc->TraversalCost({x,y}),full->TraversalCost({x,y}));}
  previous=inc;
 }
 auto oldhash=previous->platform_profile_hash();p.terrain_background_half_width_m=.2;
 auto changed=b.Derive(f.map.Snapshot(),Wheel(),p,previous),full=b.Derive(f.map.Snapshot(),Wheel(),p);
 EXPECT_NE(oldhash,changed->platform_profile_hash());
 for(int y=0;y<41;++y)for(int x=0;x<41;++x){EXPECT_EQ(changed->State({x,y}),full->State({x,y}));EXPECT_DOUBLE_EQ(changed->TraversalCost({x,y}),full->TraversalCost({x,y}));}
 }
}


TEST(TerrainObservation, IsolatedHolesRecoverWithoutChangingRawEvidence) {
 for(double r:{.025,.05,.1,.2}) for(bool legged:{false}) {
  Fixture f; f.g.resolution_m=r;
  f.z[20*41+20]=std::numeric_limits<float>::quiet_NaN();
  auto raw=f.snap(); LeggedCapability l; l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  auto e=MakePlatformElevationEvaluator(c,.2)->Evaluate(*raw,{20,20});
  EXPECT_EQ(e.state,r<=.1?IntrinsicCellState::kFree:IntrinsicCellState::kUnknown);
  EXPECT_FALSE(raw->ElevationRangeAt({20,20}));
  TraversabilityProfile p;p.terrain_background_half_width_m=.2;
  p.planar_envelope_xy_m={{-.3,-.23},{.3,-.23},{.3,.23},{-.3,.23}};
  EXPECT_EQ(FineCellEvaluator(*raw,c,p).Evaluate({20,20}).state,
    r<=.1?FineCellState::kFree:FineCellState::kUnknown);
 }
}
TEST(TerrainObservation, WideUnobservedGapsAndStepEdgesAreNotFilled) {
 for(double r:{.025,.05,.1}) for(bool legged:{false}) for(int scene=0;scene<4;++scene){
  if(scene==1) continue;
  Fixture f;f.g.resolution_m=r;const float nan=std::numeric_limits<float>::quiet_NaN();
  f.z[20*41+20]=nan;
  if(scene==0) for(int y=0;y<41;++y) for(int x=0;x<41;++x)
   if(std::abs(x-20)*r<=.20) f.z[y*41+x]=nan;
  if(scene==1) f.z[20*41+21]=nan;
  if(scene==2) for(int y=0;y<41;++y)for(int x=21;x<41;++x)f.z[y*41+x]=.1f;
  if(scene==3) f.z[20*41+21]=.3f;
  LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  EXPECT_EQ(MakePlatformElevationEvaluator(c,.2)->Evaluate(*f.snap(),{20,20}).state,IntrinsicCellState::kUnknown);
 }
}

TEST(TerrainObservation, SupportedInferredElevationCanBeEndpointAndLandingEvidence) {
 Fixture f; f.g.resolution_m=.05; f.z[20*41+20]=std::numeric_limits<float>::quiet_NaN();
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;
 p.planar_envelope_xy_m={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
 auto fine=FineTraversabilityBuilder().Derive(f.snap(),Wheel(),p);
 ASSERT_EQ(fine->State({20,20}),FineCellState::kFree);
 RequestLocalPlanningView view(fine,Pose2{},0.,{});
 EXPECT_TRUE(view.CanBeEndpoint({20,20}));
 EXPECT_TRUE(view.CanCertifyLeggedSupport({20,20},false));
 EXPECT_FALSE(fine->elevation()->ElevationRangeAt({20,20}));
 EXPECT_LT(fine->Confidence({20,20}),1.);
 EXPECT_TRUE(view.CanBeEndpoint({19,20}));
}

TEST(TerrainObservation, NoisySlopedSamplingHolesPreserveCollisionEnvelope) {
 for(double r:{.05,.1})for(bool legged:{false})for(int seed=0;seed<12;++seed){
  Fixture f;f.g.resolution_m=r;
  for(int y=0;y<41;++y)for(int x=0;x<41;++x){
   f.z[y*41+x]=float(.12*x*r+.06*y*r+((x*17+y*13+seed)%7-3)*.0005);
   if(x%3==seed%3 && y%3==(seed/3)%3) f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
  }
  LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  TraversabilityProfile p;p.terrain_background_half_width_m=legged?.2:.4;
  const double hx=legged?.34:.591,hy=legged?.165:.409;
  p.planar_envelope_xy_m={{-hx,-hy},{hx,-hy},{hx,hy},{-hx,hy}};
  auto raw=f.snap();FineCellEvaluator e(*raw,c,p);
  EXPECT_DOUBLE_EQ(e.hard_inflation_radius_m(),std::hypot(hx,hy));
  EXPECT_EQ(e.Evaluate({20,20}).state,FineCellState::kFree);
 }
}
TEST(TerrainObservation, KnownObstacleWinsOverNearbyHoles) {
 for(double r:{.05,.1})for(bool legged:{false}){
  Fixture f;f.g.resolution_m=r;
  for(int y=0;y<41;++y)for(int x=22;x<41;++x)f.z[y*41+x]=1.f;
  f.z[20*41+18]=std::numeric_limits<float>::quiet_NaN();
  LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  TraversabilityProfile p;p.terrain_background_half_width_m=legged?.2:.4;
  p.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}};
  auto raw=f.snap();EXPECT_EQ(FineCellEvaluator(*raw,c,p).Evaluate({20,20}).state,FineCellState::kBlocked);
 }
}
TEST(TerrainObservation, HoleUpdatesAndIncrementalDerivationMatchFull) {
 Fixture f;f.g.resolution_m=.05;TraversabilityProfile p;
 p.terrain_background_half_width_m=.2;p.preferred_clearance_m=.2;p.clearance_weight=2;
 p.planar_envelope_xy_m={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
 f.z[20*41+20]=std::numeric_limits<float>::quiet_NaN();
 for(int y=16;y<=24;++y)for(int x=16;x<=24;++x) f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
 FineTraversabilityBuilder b;auto previous=b.Derive(f.snap(),Wheel(),p);
 ASSERT_EQ(previous->State({20,20}),FineCellState::kUnknown);
 for(int epoch=0;epoch<3;++epoch){
  // Persistent fusion ignores NaN updates: start with a genuine missing strip,
  // observe its neighbors, disrupt the fit, then observe the center.
  for(int y=16;y<=24;++y)for(int x=16;x<=24;++x) if(x!=20 || y!=20) f.z[y*41+x]=0.f;
  f.z[20*41+21]=epoch==1?.3f:0.f;
  if(epoch==2) f.z[20*41+20]=0.f;
  auto raw=f.snap();
  EXPECT_EQ(raw->ElevationRangeAt({20,20}).has_value(),epoch==2);
  auto inc=b.Derive(raw,Wheel(),p,previous),full=b.Derive(raw,Wheel(),p);
  if(epoch==0) EXPECT_EQ(inc->State({20,20}),FineCellState::kFree);
  if(epoch==1) EXPECT_NE(inc->State({20,20}),FineCellState::kFree);
  for(int y=0;y<41;++y)for(int x=0;x<41;++x){
   EXPECT_EQ(inc->State({x,y}),full->State({x,y}));
   EXPECT_DOUBLE_EQ(inc->TraversalCost({x,y}),full->TraversalCost({x,y}));
  }
  previous=inc;
 }
}

TEST(TerrainConfidence, InferredFootprintHasCostEvenWhenCenterIsObserved) {
 Fixture f;f.g.resolution_m=.05;f.z[20*41+21]=std::numeric_limits<float>::quiet_NaN();
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;
 p.planar_envelope_xy_m={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
 p.confidence_weight=4.;auto raw=f.snap();
 auto a=FineCellEvaluator(*raw,Wheel(),p).Evaluate({20,20});
 EXPECT_EQ(a.state,FineCellState::kFree);EXPECT_GT(a.confidence,0.);EXPECT_LT(a.confidence,1.);
 EXPECT_NEAR(a.traversal_cost,4.*(1.-a.confidence),1e-12);
 p.confidence_weight=0.;auto b=FineCellEvaluator(*raw,Wheel(),p).Evaluate({20,20});
 EXPECT_EQ(a.state,b.state);EXPECT_DOUBLE_EQ(b.confidence,a.confidence);EXPECT_DOUBLE_EQ(b.traversal_cost,0.);
}
TEST(TerrainConfidence, TwoByTwoSamplingHoleUsesNeighborPlane) {
 Fixture f;f.g.resolution_m=.05;
 for(int y=20;y<=21;++y)for(int x=20;x<=21;++x)f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
 for(bool legged:{false}){
  LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  auto e=MakePlatformElevationEvaluator(c,.2)->Evaluate(*f.snap(),{20,20});
  EXPECT_EQ(e.state,IntrinsicCellState::kFree);EXPECT_GT(e.confidence,0.);EXPECT_LT(e.confidence,1.);
 }
}
TEST(TerrainConfidence, FullyObservedFlatIsHighAndInvalidWeightRejected) {
 Fixture f;auto raw=f.snap();TraversabilityProfile p;
 p.planar_envelope_xy_m={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
 auto e=FineCellEvaluator(*raw,Wheel(),p).Evaluate({20,20});
 EXPECT_DOUBLE_EQ(e.confidence,1.);EXPECT_DOUBLE_EQ(e.traversal_cost,0.);
 p.confidence_weight=-1.;EXPECT_THROW(FineCellEvaluator(*raw,Wheel(),p),std::invalid_argument);
}

TEST(TerrainConfidence, PlannerPrefersObservedDetourAndKeepsSoftRegionReachable) {
 Fixture f;f.g.resolution_m=.05;
 for(int y=17;y<=23;y+=3)for(int x=14;x<=26;x+=3) f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;
 p.planar_envelope_xy_m={{-.05,-.05},{.05,-.05},{.05,.05},{-.05,.05}};
 const Pose2 start{.position_m={.425,1.025}};const LocalTarget goal{.center={1.625,1.025},.is_final_goal=true};
 auto raw=f.snap();double deviations[2]{};
 for(int i=0;i<2;++i){
  p.confidence_weight=i?8.:0.;auto fine=FineTraversabilityBuilder().Derive(raw,Wheel(),p);
  RequestLocalPlanningView view(fine,start,0.,{});
  auto result=WheelLocalPlanner().Plan(view,start,goal,SteadyClock::time_point::max(),{});
  ASSERT_EQ(result.status,LocalPlanResult::Status::kPlanFound);
  for(const auto& v:result.path) deviations[i]=std::max(deviations[i],std::abs(v.pose.position_m.y-1.025));
 }
 EXPECT_GT(deviations[1],deviations[0]+.1);
}
TEST(TerrainConfidence, WeightChangeInvalidatesDerivedCostCache) {
 Fixture f;f.g.resolution_m=.05;f.z[20*41+21]=std::numeric_limits<float>::quiet_NaN();
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;p.confidence_weight=0.;
 p.planar_envelope_xy_m={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
 auto raw=f.snap();FineTraversabilityBuilder b;auto old=b.Derive(raw,Wheel(),p);
 p.confidence_weight=4.;auto changed=b.Derive(raw,Wheel(),p,old),full=b.Derive(raw,Wheel(),p);
 EXPECT_NE(old->platform_profile_hash(),changed->platform_profile_hash());
 EXPECT_NEAR(changed->TraversalCost({20,20}),4.*(1.-changed->Confidence({20,20})),1e-6);
 EXPECT_GT(old->Confidence({20,20}),0.);EXPECT_LT(old->Confidence({20,20}),1.);
 EXPECT_DOUBLE_EQ(changed->Confidence({20,20}),old->Confidence({20,20}));
 EXPECT_DOUBLE_EQ(changed->TraversalCost({20,20}),full->TraversalCost({20,20}));
}



TEST(TerrainConfidence, NewObservationRaisesConfidenceEvenWithZeroCostWeight) {
 Fixture f;f.g.resolution_m=.05;f.z[20*41+21]=std::numeric_limits<float>::quiet_NaN();
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;p.confidence_weight=0.;
 p.planar_envelope_xy_m={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
 FineTraversabilityBuilder b;auto old=b.Derive(f.snap(),Wheel(),p);
 ASSERT_GT(old->Confidence({20,20}),0.);ASSERT_LT(old->Confidence({20,20}),1.);
 f.z[20*41+21]=0.f;auto raw=f.snap();auto updated=b.Derive(raw,Wheel(),p,old),full=b.Derive(raw,Wheel(),p);
 EXPECT_EQ(old->State({20,20}),updated->State({20,20}));
 EXPECT_DOUBLE_EQ(old->TraversalCost({20,20}),updated->TraversalCost({20,20}));
 EXPECT_DOUBLE_EQ(updated->Confidence({20,20}),1.);
 EXPECT_FALSE(updated->changed_tiles().empty());
 for(int y=0;y<41;++y)for(int x=0;x<41;++x) EXPECT_DOUBLE_EQ(updated->Confidence({x,y}),full->Confidence({x,y}));
}
TEST(TerrainConfidence, SoftCostDoesNotCloseTheOnlyCorridor) {
 Fixture f;f.g.resolution_m=.05;
 for(int y=0;y<41;++y)for(int x=0;x<41;++x){
  if(y<=14 || y>=26) f.z[y*41+x]=1.f;
  else if(x>=14 && x<=26 && x%3==0 && y%3==2) f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
 }
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;p.confidence_weight=100.;
 p.planar_envelope_xy_m={{-.05,-.05},{.05,-.05},{.05,.05},{-.05,.05}};
 auto fine=FineTraversabilityBuilder().Derive(f.snap(),Wheel(),p);
 const Pose2 start{.position_m={.425,1.025}};RequestLocalPlanningView view(fine,start,0.,{});
 auto result=WheelLocalPlanner().Plan(view,start,{.center={1.625,1.025},.is_final_goal=true},SteadyClock::time_point::max(),{});
 ASSERT_EQ(result.status,LocalPlanResult::Status::kPlanFound);
 bool crosses_inference=false;
 for(const auto& v:result.raw_path){
  const GridIndex cell{int(v.pose.position_m.x/.05),int(v.pose.position_m.y/.05)};
  crosses_inference |= fine->Confidence(cell)>0. && fine->Confidence(cell)<1.;
 }
 EXPECT_TRUE(crosses_inference);
}

TEST(TerrainConfidence, SmallSupportedHoleDoesNotFitAcrossDistantStep) {
 Fixture f;f.g.resolution_m=.05;f.z[20*41+20]=std::numeric_limits<float>::quiet_NaN();
 for(int y=0;y<41;++y)for(int x=22;x<41;++x) f.z[y*41+x]=.1f;
 const auto e=MakePlatformElevationEvaluator(Wheel(),.2)->Evaluate(*f.snap(),{20,20});
 EXPECT_EQ(e.state,IntrinsicCellState::kFree);EXPECT_GT(e.confidence,0.);EXPECT_LT(e.confidence,1.);
}

TEST(TerrainContinuousConfidence, TwentyCmSamplesSupportFiveCmGround) {
 for(bool legged:{false}) {
  Fixture f;f.g.resolution_m=.05;
  std::fill(f.z.begin(),f.z.end(),std::numeric_limits<float>::quiet_NaN());
  for(int y=0;y<41;y+=4)for(int x=0;x<41;x+=4)f.z[y*41+x]=float(.05*x*.05+.02*y*.05);
  auto raw=f.snap();LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  for(int y=18;y<22;++y)for(int x=18;x<22;++x){
   auto e=MakePlatformElevationEvaluator(c,.2)->Evaluate(*raw,{x,y});
   EXPECT_EQ(e.state,IntrinsicCellState::kFree)<<x<<","<<y;
   EXPECT_GT(e.confidence,0.);EXPECT_LT(e.confidence,1.);
  }
  TraversabilityProfile p;p.terrain_background_half_width_m=.2;
  p.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}};
  auto e=FineCellEvaluator(*raw,c,p).Evaluate({20,20});
  EXPECT_EQ(e.state,FineCellState::kFree);EXPECT_GT(e.confidence,0.);EXPECT_LT(e.confidence,1.);
  EXPECT_FALSE(raw->ElevationRangeAt({19,19}));
 }
}
TEST(TerrainContinuousConfidence, EvidenceQualityProducesDifferentContinuousScores) {
 double scores[3]{};
 for(int scene=0;scene<3;++scene){
  Fixture f;f.g.resolution_m=.05;
  for(int y=0;y<41;++y)for(int x=0;x<41;++x){
   if(scene==2 && (x%4 || y%4)) f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
   else if(scene==1) f.z[y*41+x]=float(((x*17+y*7)%5-2)*.003);
  }
  f.z[20*41+20]=std::numeric_limits<float>::quiet_NaN();
  auto e=MakePlatformElevationEvaluator(Wheel(),.2)->Evaluate(*f.snap(),{20,20});
  ASSERT_EQ(e.state,IntrinsicCellState::kFree);scores[scene]=e.confidence;
  EXPECT_GT(e.confidence,0.);EXPECT_LT(e.confidence,1.);
 }
 EXPECT_GT(scores[0],scores[1]);EXPECT_GT(scores[0],scores[2]);
 EXPECT_NE(scores[0],.5);
}

TEST(TerrainContinuousConfidence, FractionalEvidenceUpdateInvalidatesZeroWeightSnapshot) {
 Fixture f;f.g.resolution_m=.05;
 std::fill(f.z.begin(),f.z.end(),std::numeric_limits<float>::quiet_NaN());
 for(int y=0;y<41;y+=4)for(int x=0;x<41;x+=4)f.z[y*41+x]=0.f;
 TraversabilityProfile p;p.terrain_background_half_width_m=.2;p.confidence_weight=0.;
 p.planar_envelope_xy_m={{-.001,-.001},{.001,-.001},{.001,.001},{-.001,.001}};
 FineTraversabilityBuilder b;auto old=b.Derive(f.snap(),Wheel(),p);
 ASSERT_EQ(old->State({19,19}),FineCellState::kFree);
 for(int y=0;y<41;y+=2)for(int x=0;x<41;x+=2)f.z[y*41+x]=0.f;
 auto raw=f.snap();auto inc=b.Derive(raw,Wheel(),p,old),full=b.Derive(raw,Wheel(),p);
 EXPECT_GT(inc->Confidence({19,19}),old->Confidence({19,19}));
 EXPECT_LT(inc->Confidence({19,19}),1.);
 EXPECT_EQ(inc->TraversalCost({19,19}),old->TraversalCost({19,19}));
 EXPECT_FALSE(raw->ElevationRangeAt({19,19}));EXPECT_FALSE(inc->changed_tiles().empty());
 for(int y=0;y<41;++y)for(int x=0;x<41;++x){
  EXPECT_EQ(inc->State({x,y}),full->State({x,y}));
  EXPECT_DOUBLE_EQ(inc->Confidence({x,y}),full->Confidence({x,y}));
  EXPECT_DOUBLE_EQ(inc->TraversalCost({x,y}),full->TraversalCost({x,y}));
 }
}
TEST(TerrainContinuousConfidence, SparseOneSidedLineAndConflictingEvidenceStayUnknown) {
 for(int scene=0;scene<3;++scene){
  Fixture f;f.g.resolution_m=.05;
  std::fill(f.z.begin(),f.z.end(),std::numeric_limits<float>::quiet_NaN());
  for(int y=0;y<41;y+=4)for(int x=0;x<41;x+=4){
   if((scene==0 && x<20)||(scene==1 && y!=20)) continue;
   f.z[y*41+x]=scene==2 && x>=20?.6f:0.f;
  }
  auto raw=f.snap();auto e=MakePlatformElevationEvaluator(Wheel(),.2)->Evaluate(*raw,{19,19});
  if(scene<2) { EXPECT_EQ(e.state,IntrinsicCellState::kUnknown);EXPECT_DOUBLE_EQ(e.confidence,0.); }
  else EXPECT_NE(e.state,IntrinsicCellState::kFree); // reliable steep evidence can be BLOCKED
 }
}
TEST(TerrainContinuousConfidence, FineTileRejectsInvalidConfidence) {
 FineTraversabilityTile::StateArray s{};FineTraversabilityTile::CostArray costs{};
 FineTraversabilityTile::ConfidenceArray confidence{};
 for(float v:{-0.1f,1.1f,std::numeric_limits<float>::quiet_NaN()}){
  confidence[0]=v;EXPECT_THROW(FineTraversabilityTile(s,costs,confidence),std::invalid_argument);
 }
}
TEST(TerrainContinuousConfidence, SparseMeasuredStepAndPitCannotBecomeFreeBridge) {
 for(bool legged:{false})for(double r:{.05,.1})for(float height:{-1.f,1.f}){
  Fixture f;f.g.resolution_m=r;const int stride=std::lround(.20/r);
  std::fill(f.z.begin(),f.z.end(),std::numeric_limits<float>::quiet_NaN());
  for(int y=0;y<41;y+=stride)for(int x=0;x<41;x+=stride)f.z[y*41+x]=x>=20?height:0.f;
  LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  auto raw=f.snap();TraversabilityProfile p;p.terrain_background_half_width_m=.2;
  p.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}};
  EXPECT_NE(FineCellEvaluator(*raw,c,p).Evaluate({20,20}).state,FineCellState::kFree);
 }
}

TEST(TerrainSeparation, SparseSlopeUsesPlatformLimitAfterObservation) {
 Fixture f;f.g.resolution_m=.05;
 for(int y=0;y<41;++y)for(int x=0;x<41;++x)
  f.z[y*41+x]=(x%4 || y%4)?std::numeric_limits<float>::quiet_NaN():float(std::tan(.45)*x*.05);
 auto raw=f.snap();auto wheel=MakePlatformElevationEvaluator(Wheel(),.2)->Evaluate(*raw,{18,18});
 EXPECT_EQ(wheel.state,IntrinsicCellState::kBlocked);
 EXPECT_NEAR(wheel.slope_rad,.45,1e-5);EXPECT_GT(wheel.confidence,0.);

}
TEST(TerrainSeparation, CrossTileObservationAndGeometryGrowthMatchFullWithZeroConfidenceWeight) {
 GridGeometry g{.frame_id="map",.width=272,.height=41,.resolution_m=.05};
 std::vector<float> values(g.CellCount(),std::numeric_limits<float>::quiet_NaN());
 for(int y=0;y<41;y+=4)for(int x=0;x<272;x+=4)values[y*272+x]=0.f;
 PersistentElevationMap map;FineTraversabilityBuilder builder;TraversabilityProfile p;
 p.planar_envelope_xy_m={{-.05,-.05},{.05,-.05},{.05,.05},{-.05,.05}};
 p.terrain_background_half_width_m=.2;p.preferred_clearance_m=.2;p.confidence_weight=0.;
 (void)map.Apply({.geometry=g,.elevation_m=values,.map_from_source={.parent_frame="map",.child_frame="map"}});
 auto previous=builder.Derive(map.Snapshot(),Wheel(),p);
 for(int epoch=0;epoch<2;++epoch) {
  GridGeometry update{.frame_id="map",.width=1,.height=1,.resolution_m=.05,
                      .origin_m={epoch==0?12.8:14.,1.,0.}};
  const std::vector<float> update_values{.015f};
  (void)map.Apply({.geometry=update,.elevation_m=update_values,.map_from_source={.parent_frame="map",.child_frame="map"}});
  auto raw=map.Snapshot();auto inc=builder.Derive(raw,Wheel(),p,previous),full=builder.Derive(raw,Wheel(),p);
  for(auto y=raw->geometry().min_inclusive().y;y<raw->geometry().max_exclusive().y;++y)
   for(auto x=raw->geometry().min_inclusive().x;x<raw->geometry().max_exclusive().x;++x){
    EXPECT_EQ(inc->State({x,y}),full->State({x,y}));
    EXPECT_DOUBLE_EQ(inc->Confidence({x,y}),full->Confidence({x,y}));
    EXPECT_DOUBLE_EQ(inc->TraversalCost({x,y}),full->TraversalCost({x,y}));
   }
  EXPECT_LE(full->metrics().ground_cells_estimated,281u*41u);
  previous=inc;
 }
}

TEST(TerrainSeparation, MissingCentralReturnsDoNotHideSupportedOverLimitSteps) {
 for(bool legged:{false})for(GridIndex hole:{GridIndex{19,19},GridIndex{20,20}}) {
  Fixture f;f.g.resolution_m=.05;
  for(int y=0;y<41;++y)for(int x=20;x<41;++x)f.z[y*41+x]=.7f;
  f.z[hole.y*41+hole.x]=std::numeric_limits<float>::quiet_NaN();
  LeggedCapability l;l.maximum_slope_rad=.52;l.maximum_step_height_m=.5;
  PlatformCapability c=legged?PlatformCapability(l):PlatformCapability(Wheel());
  EXPECT_EQ(MakePlatformElevationEvaluator(c,.2)->Evaluate(*f.snap(),{20,20}).state,IntrinsicCellState::kBlocked);
 }
}

TEST(TerrainSeparation, MissingEntireCentralPatchStaysUnknown) {
 Fixture f;f.g.resolution_m=.2;
 for(int y=0;y<41;++y)for(int x=0;x<41;++x)f.z[y*41+x]=.2f*x;
 for(int y=19;y<=21;++y)for(int x=19;x<=21;++x)f.z[y*41+x]=std::numeric_limits<float>::quiet_NaN();
 auto e=MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*f.snap(),{20,20});
 EXPECT_EQ(e.state,IntrinsicCellState::kUnknown);
 EXPECT_TRUE(std::isfinite(e.relief_m));
}
