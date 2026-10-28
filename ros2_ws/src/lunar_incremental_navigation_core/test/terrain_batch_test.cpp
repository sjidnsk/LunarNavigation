#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "lunar_incremental_navigation_core/terrain_batch.hpp"
namespace nav=lunar::incremental_navigation;
TEST(TerrainBatch, MatchesScalarOnConstantsSlopeStepsHolesAndIsolatedReturns) {
  for (double r:{.01,.05,.1}) for (int scene=0;scene<8;++scene) {
    constexpr int n=21;std::vector<float> values(n*n);
    for(int y=0;y<n;++y)for(int x=0;x<n;++x) {
      float z=scene==1?.3f:scene==2?float(.15*x*r):0.f;
      if(scene==6)z=float(.02*y*r);
      if(scene==7)z=float(.02*(x+y)*r);
      if(scene==3 && x>=10)z=.4f;
      if(scene==4 && x==10 && y==10)z=std::numeric_limits<float>::quiet_NaN();
      if(scene==5 && x==10 && y==10)z=.4f;
      values[y*n+x]=z;
    }
    nav::PersistentElevationMap map;
    const auto result=map.Apply({.geometry={.frame_id="map",.width=n,.height=n,
        .resolution_m=r,.origin_m={0,0,0}},.elevation_m=values,
        .map_from_source={.parent_frame="map",.child_frame="map"}});
    ASSERT_EQ(result.status,nav::ElevationUpdateResult::Status::kApplied);
    auto view=map.Snapshot();nav::WheeledCapability wheel;wheel.maximum_slope_rad=.6;wheel.maximum_local_obstacle_relief_m=.2;wheel.minimum_underbody_clearance_m=.2;
    auto scalar=nav::MakePlatformElevationEvaluator(wheel);
    auto batch=nav::MakeSnapshotElevationEvaluator(*view,wheel);
    nav::SnapshotTerrainClassifier classifier(*view,wheel);
    for(int y=0;y<n;y+=2)for(int x=0;x<n;x+=2) {
      SCOPED_TRACE(::testing::Message()<<"r="<<r<<" scene="<<scene<<" x="<<x<<" y="<<y);
      auto a=scalar->Evaluate(*view,{x,y});auto b=batch->Evaluate(*view,{x,y});
      EXPECT_EQ(a.state,classifier.Classify({x,y}));
      EXPECT_EQ(a.state,b.state);EXPECT_DOUBLE_EQ(a.slope_rad,b.slope_rad);
      EXPECT_DOUBLE_EQ(a.relief_m,b.relief_m);EXPECT_DOUBLE_EQ(a.confidence,b.confidence);
    }
    nav::ElevationSnapshot other;
    EXPECT_THROW(batch->Evaluate(other,{1,1}),std::invalid_argument);
  }
}

TEST(TerrainBatch, FreeCertificateRejectsThresholdsHolesRangesAndZeroCapabilities) {
  class View final:public nav::ElevationRangeView {
   public:
    nav::SparseGridGeometry g{"map",.01,{0,0,0},{0,0},{23,19}};
    std::vector<std::optional<nav::ElevationRange>> cells;
    const nav::SparseGridGeometry& geometry() const noexcept override{return g;}
    std::optional<nav::ElevationRange> ElevationRangeAt(nav::GridIndex i) const noexcept override {
      if(!g.Contains(i))return {};return cells[i.y*23+i.x];
    }
  };
  for(int scene=0;scene<10;++scene) {
    View v;v.cells.resize(23*19);nav::WheeledCapability wheel;wheel.maximum_slope_rad=.6;wheel.maximum_local_obstacle_relief_m=.2;wheel.minimum_underbody_clearance_m=.2;
    for(int y=0;y<19;++y)for(int x=0;x<23;++x) {
      const double slope=scene==2?wheel.maximum_slope_rad:scene==3?wheel.maximum_slope_rad+1e-6:scene==8?wheel.maximum_slope_rad-1e-8:.02;
      float z=float(.3+std::tan(slope)*x*.01+(scene==8?0.:.01*y*.01));
      if(scene==1)z+=float(1e-6*std::sin(x*1.7+y*.3));
      v.cells[y*23+x]=nav::ElevationRange{z,z};
    }
    if(scene==4)v.cells[9*23+11].reset();
    if(scene==5)v.cells[9*23+11]->max_m+=1e-5f;
    if(scene==6)wheel.maximum_slope_rad=0.;
    if(scene==7)wheel.maximum_local_obstacle_relief_m=0.;
    if(scene==9)wheel.minimum_underbody_clearance_m=0.;
    auto scalar=nav::MakePlatformElevationEvaluator(wheel);
    nav::SnapshotTerrainClassifier classifier(v,wheel);
    for(int y=0;y<19;++y)for(int x=0;x<23;++x) {
      SCOPED_TRACE(::testing::Message()<<"scene="<<scene<<" x="<<x<<" y="<<y);
      EXPECT_EQ(scalar->Evaluate(v,{x,y}).state,classifier.Classify({x,y}));
    }
    if(scene<2)EXPECT_GT(classifier.certified_free_cells(),0U);
    else EXPECT_EQ(classifier.certified_free_cells(),0U);
  }
}

TEST(TerrainBatch, ClippedThinRectanglesAndLeggedRemainScalarEquivalent) {
  for(auto dims:{std::pair<int,int>{3,61},{61,3},{4,7}}) {
    auto [w,h]=dims;std::vector<float> z(w*h);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x)z[y*w+x]=float(.02*x*.01+.01*y*.01);
    nav::PersistentElevationMap map;
    ASSERT_EQ(map.Apply({.geometry={.frame_id="map",.width=std::size_t(w),.height=std::size_t(h),
        .resolution_m=.01,.origin_m={0,0,0}},.elevation_m=z,
        .map_from_source={.parent_frame="map",.child_frame="map"}}).status,nav::ElevationUpdateResult::Status::kApplied);
    auto view=map.Snapshot();
    for(nav::PlatformCapability capability:{nav::PlatformCapability{nav::WheeledCapability{}},nav::PlatformCapability{nav::LeggedCapability{}}}) {
      auto scalar=nav::MakePlatformElevationEvaluator(capability);
      nav::SnapshotTerrainClassifier classifier(*view,capability);
      for(int y=0;y<h;++y)for(int x=0;x<w;++x)
        EXPECT_EQ(scalar->Evaluate(*view,{x,y}).state,classifier.Classify({x,y}));
      if(std::holds_alternative<nav::LeggedCapability>(capability))EXPECT_EQ(classifier.certified_free_cells(),0U);
    }
  }
}

TEST(TerrainBatch, LocalCertificateKeepsCompleteOutlierAndHoleDependencies) {
  for(bool hole:{false,true}) {
    constexpr int n=81;std::vector<float> z(n*n);
    for(int y=0;y<n;++y)for(int x=0;x<n;++x)z[y*n+x]=float(.02*x*.01+.01*y*.01);
    z[40*n+40]=hole?std::numeric_limits<float>::quiet_NaN():z[40*n+40]+.4f;
    nav::PersistentElevationMap map;
    ASSERT_EQ(map.Apply({.geometry={.frame_id="map",.width=n,.height=n,.resolution_m=.01,.origin_m={0,0,0}},
      .elevation_m=z,.map_from_source={.parent_frame="map",.child_frame="map"}}).status,nav::ElevationUpdateResult::Status::kApplied);
    auto view=map.Snapshot();nav::WheeledCapability wheel;
    wheel.maximum_slope_rad=.6;wheel.maximum_local_obstacle_relief_m=.2;wheel.minimum_underbody_clearance_m=.2;
    auto scalar=nav::MakePlatformElevationEvaluator(wheel);nav::SnapshotTerrainClassifier classifier(*view,wheel);
    for(int y:{0,2,7,8,39,40,41,72,73,78,80})for(int x:{0,2,7,8,39,40,41,72,73,78,80})
      EXPECT_EQ(scalar->Evaluate(*view,{x,y}).state,classifier.Classify({x,y}));
    EXPECT_GT(classifier.certified_free_cells(),0U);
  }
}
