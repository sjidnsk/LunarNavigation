#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "lunar_incremental_navigation_core/terrain_batch.hpp"
namespace nav=lunar::incremental_navigation;
TEST(TerrainBatch, MatchesScalarOnConstantsSlopeStepsHolesAndIsolatedReturns) {
  for (double r:{.01,.05,.1}) for (int scene=0;scene<6;++scene) {
    constexpr int n=21;std::vector<float> values(n*n);
    for(int y=0;y<n;++y)for(int x=0;x<n;++x) {
      float z=scene==1?.3f:scene==2?float(.15*x*r):0.f;
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
    auto view=map.Snapshot();nav::WheeledCapability wheel;
    auto scalar=nav::MakePlatformElevationEvaluator(wheel);
    auto batch=nav::MakeSnapshotElevationEvaluator(*view,wheel);
    for(int y=0;y<n;y+=2)for(int x=0;x<n;x+=2) {
      SCOPED_TRACE(::testing::Message()<<"r="<<r<<" scene="<<scene<<" x="<<x<<" y="<<y);
      auto a=scalar->Evaluate(*view,{x,y});auto b=batch->Evaluate(*view,{x,y});
      EXPECT_EQ(a.state,b.state);EXPECT_DOUBLE_EQ(a.slope_rad,b.slope_rad);
      EXPECT_DOUBLE_EQ(a.relief_m,b.relief_m);EXPECT_DOUBLE_EQ(a.confidence,b.confidence);
    }
    nav::ElevationSnapshot other;
    EXPECT_THROW(batch->Evaluate(other,{1,1}),std::invalid_argument);
  }
}
