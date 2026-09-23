#include <gtest/gtest.h>
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
TEST(TerrainWindow, CoverageUsesPhysicalWindow){
 Fixture f;std::fill(f.z.begin(),f.z.end(),std::numeric_limits<float>::quiet_NaN());
 for(int y=18;y<=22;++y)for(int x=18;x<=22;++x)f.z[y*41+x]=0;
 auto raw=f.snap();
 EXPECT_EQ(MakePlatformElevationEvaluator(Wheel())->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kFree);
 EXPECT_EQ(MakePlatformElevationEvaluator(Wheel(),.4)->Evaluate(*raw,{20,20}).state,IntrinsicCellState::kUnknown);
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
