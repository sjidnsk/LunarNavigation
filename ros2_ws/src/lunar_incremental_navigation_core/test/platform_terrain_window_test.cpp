#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include "lunar_incremental_navigation_core/platform_terrain_window.hpp"
#include "map/terrain_window_statistics.hpp"
#include <deque>
namespace lunar::incremental_navigation { namespace {
class View final: public ElevationRangeView {
 public:
 SparseGridGeometry g; std::vector<float> h;
 View(double r):g("map",r,{}, {0,0},{61,61}),h(61*61,0.f){}
 const SparseGridGeometry& geometry()const noexcept override{return g;}
 std::optional<ElevationRange> ElevationRangeAt(GridIndex i)const noexcept override {
  if(!g.Contains(i))return {};auto z=h[i.y*61+i.x];if(!std::isfinite(z))return {};return ElevationRange{z,z};
 }
 std::optional<LocalTerrainMeasurements> TerrainMeasurementsAt(GridIndex)const noexcept override{return LocalTerrainMeasurements{true,true,1.4,9.,9.};}
 void Plane(double angle,double offset=0.){for(int y=0;y<61;++y)for(int x=0;x<61;++x)h[y*61+x]=offset+std::tan(angle)*(x-30)*g.resolution_m();}
};
TEST(PlatformTerrainWindow, PlatformDimensionsAndRadii){
 WheeledCapability w;w.body_extent_m={1.182,.818,1.3};LeggedCapability l;l.body_extent_m={.68,.33,.35};
 EXPECT_DOUBLE_EQ(PlatformTerrainLongSide(w),1.182);EXPECT_DOUBLE_EQ(PlatformTerrainLongSide(l),.68);
 EXPECT_EQ(PlatformTerrainWindowRadiusCells(1.182,.2),3);EXPECT_EQ(PlatformTerrainWindowRadiusCells(1.182,.05),12);
 EXPECT_EQ(PlatformTerrainWindowRadiusCells(.68,.2),2);EXPECT_EQ(PlatformTerrainWindowRadiusCells(.68,.05),7);
 EXPECT_EQ(PlatformTerrainWindowRadiusCells(1.182,.05,true),4);
 EXPECT_THROW(PlatformTerrainWindowRadiusCells(0.,.2),std::invalid_argument);
}
TEST(PlatformTerrainWindow, PlaneUsesRawHeightAtPhysicalScale){
 for(double L:{1.182,.68})for(double r:{.05,.1,.2})for(double degrees:{0.,19.9,20.,20.1}){
  View v(r);v.Plane(degrees*std::numbers::pi/180.);auto m=MeasurePlatformTerrain(v,{30,30},L);
  EXPECT_TRUE(m.center_known);EXPECT_TRUE(m.neighborhood_complete);EXPECT_NEAR(m.slope_rad,degrees*std::numbers::pi/180.,1e-6);
  EXPECT_NEAR(m.relief_m,0.,1e-6);EXPECT_NEAR(m.positive_rise_m,0.,1e-6);
 }
}
TEST(PlatformTerrainWindow, StepAndIsolatedPeakAreNotAbsorbed){
 for(double L:{1.182,.68})for(double r:{.05,.1,.2})for(float z:{.199f,.201f,.499f,.501f}){
  View v(r);for(int y=0;y<61;++y)for(int x=31;x<61;++x)v.h[y*61+x]=z;
  auto step=MeasurePlatformTerrain(v,{30,30},L);EXPECT_NEAR(step.relief_m,z,1e-6);EXPECT_NEAR(step.positive_rise_m,z,1e-6);
  View p(r);p.h[30*61+31]=z;auto peak=MeasurePlatformTerrain(p,{30,30},L);EXPECT_NEAR(peak.relief_m,z,1e-6);EXPECT_NEAR(peak.positive_rise_m,z,1e-6);
 }
}
TEST(PlatformTerrainWindow, MissingCenterIsUnknownButWindowHolesAreNotVetoes){
 View v(.1);v.h[30*61+30]=std::numeric_limits<float>::quiet_NaN();EXPECT_FALSE(MeasurePlatformTerrain(v,{30,30},1.182).center_known);
 v.h[30*61+30]=0.;v.h[30*61+31]=std::numeric_limits<float>::quiet_NaN();EXPECT_TRUE(MeasurePlatformTerrain(v,{30,30},1.182).neighborhood_complete);
 EXPECT_TRUE(MeasurePlatformTerrain(v,{0,0},1.182).neighborhood_complete);
}
TEST(PlatformTerrainWindow, BatchAndPointAgree){
 View v(.1);v.Plane(.21,1000.);v.h[29*61+31]+=.25f;
 auto batch=MeasurePlatformTerrainBatch(v,1.182,{27,27},{34,34});
 for(int y=27;y<34;++y)for(int x=27;x<34;++x){auto a=MeasurePlatformTerrain(v,{x,y},1.182);auto b=batch.At({x,y});ASSERT_TRUE(b);EXPECT_NEAR(a.slope_rad,b->slope_rad,1e-9);EXPECT_NEAR(a.relief_m,b->relief_m,1e-9);EXPECT_EQ(a.neighborhood_complete,b->neighborhood_complete);}
 EXPECT_EQ(batch.At({0,0}),nullptr);
}
TEST(PlatformTerrainWindow, SlopedStepRetainsItsHeight){
 for(double r:{.05,.1,.2}){View v(r);v.Plane(10.*std::numbers::pi/180.);for(int y=0;y<61;++y)for(int x=31;x<61;++x)v.h[y*61+x]+=.201f;auto m=MeasurePlatformTerrain(v,{30,30},1.182);EXPECT_NEAR(m.relief_m,.201,1e-6);EXPECT_NEAR(m.positive_rise_m,.201,1e-6);}
}
TEST(PlatformTerrainWindow, LargeWorldCoordinatesDoNotChangeFit){
 View a(.05),b(.05);a.Plane(.2,1000.);b.Plane(.2,1000.);b.g=SparseGridGeometry("map",.05,{1.e12,-1.e12,0.},{0,0},{61,61});
 auto x=MeasurePlatformTerrain(a,{30,30},1.182),y=MeasurePlatformTerrain(b,{30,30},1.182);EXPECT_DOUBLE_EQ(x.slope_rad,y.slope_rad);EXPECT_DOUBLE_EQ(x.relief_m,y.relief_m);
}
TEST(PlatformTerrainWindow, FactoryReusesThresholdsAndBindsView){
 View v(.1),other(.1);WheeledCapability w;w.body_extent_m={1.182,.818,1.3};w.maximum_slope_rad=.35;w.maximum_local_obstacle_relief_m=.2;w.minimum_underbody_clearance_m=.21;
 auto evaluator=MakePlatformWindowElevationEvaluator(v,w);EXPECT_EQ(evaluator->Evaluate(v,{30,30}).state,IntrinsicCellState::kFree);EXPECT_THROW(evaluator->Evaluate(other,{30,30}),std::invalid_argument);
}
TEST(PlatformTerrainWindow, RejectsUnrepresentableBatchSpanBeforeSubtraction){
 View v(.1);
 EXPECT_THROW(MeasurePlatformTerrainBatch(v,1.182,{std::numeric_limits<std::int64_t>::min(),0},{std::numeric_limits<std::int64_t>::max(),1}),std::overflow_error);
}
TEST(PlatformTerrainWindow, FactoryNearInt64MaximumHasCheckedBlockEnd){
 class ExtremeView final:public ElevationRangeView {
  SparseGridGeometry g_{"map",.1,{}, {std::numeric_limits<std::int64_t>::max()-63,0},{std::numeric_limits<std::int64_t>::max(),61}};
  public:const SparseGridGeometry& geometry()const noexcept override{return g_;}
  std::optional<ElevationRange> ElevationRangeAt(GridIndex i)const noexcept override{return g_.Contains(i)?std::optional<ElevationRange>(ElevationRange{0.f,0.f}):std::nullopt;}
 } view;
 WheeledCapability w;w.body_extent_m={1.182,.818,1.3};w.maximum_slope_rad=.35;w.maximum_local_obstacle_relief_m=.2;w.minimum_underbody_clearance_m=.21;
 auto evaluator=MakePlatformWindowElevationEvaluator(view,w);
 // halo超出int64域的样本是missing；有效的已观测中心不能因表达边界抛异常。
 GridIndex index{std::numeric_limits<std::int64_t>::max()-1,30};
 EXPECT_NO_THROW({auto m=MeasurePlatformTerrain(view,index,1.182);EXPECT_TRUE(m.center_known);EXPECT_TRUE(m.neighborhood_complete);});
 EXPECT_NO_THROW(EXPECT_EQ(evaluator->Evaluate(view,index).state,IntrinsicCellState::kFree));
}
TEST(PlatformTerrainWindow, BatchLookupChecksFarIndexWithoutSignedSubtraction){
 PlatformTerrainMeasurementBatch batch{{std::numeric_limits<std::int64_t>::min(),0},1,1,{LocalTerrainMeasurements{}}};
 EXPECT_EQ(batch.At({std::numeric_limits<std::int64_t>::max(),0}),nullptr);
}
TEST(PlatformTerrainWindow, FactorySeparatesLeggedStepFromGroundSlope){
 LeggedCapability cap;cap.body_extent_m={.68,.33,.35};cap.maximum_slope_rad=20.*std::numbers::pi/180.;cap.maximum_step_height_m=.5;cap.maximum_gap_width_m=.3;cap.minimum_body_clearance_m=.3;
 for(double r:{.05,.1,.2})for(double angle:{0.,19.9*std::numbers::pi/180.,20.1*std::numbers::pi/180.})for(float z:{.499f,.501f}){
  View v(r);v.Plane(angle);for(int y=0;y<61;++y)for(int x=31;x<61;++x)v.h[y*61+x]+=z;
  auto evaluator=MakePlatformWindowElevationEvaluator(v,cap);auto result=evaluator->Evaluate(v,{30,30});
  auto expected=z<.5f&&angle<20.*std::numbers::pi/180.?IntrinsicCellState::kFree:IntrinsicCellState::kBlocked;
  EXPECT_EQ(result.state,expected)<<"r="<<r<<" angle="<<angle<<" step="<<z;
 }
}
TEST(PlatformTerrainWindow, NegativePeakAndTwoStepsRetainRelief){
 for(double r:{.05,.1,.2}){
  View v(r);v.h[30*61+31]=-.501f;auto m=MeasurePlatformTerrain(v,{30,30},.68);EXPECT_NEAR(m.relief_m,.501,1e-6);
  View steps(r);for(int y=0;y<61;++y){steps.h[y*61+29]=-.3f;steps.h[y*61+31]=.3f;}auto twice=MeasurePlatformTerrain(steps,{30,30},.68);EXPECT_NEAR(twice.relief_m,.6,1e-6);
 }
}
TEST(PlatformTerrainWindow, DiagonalPlaneAndStepUseAllWindowRows){
 for(double r:{.05,.1,.2}){
  View v(r);const double tangent=std::tan(19.9*std::numbers::pi/180.);
  for(int y=0;y<61;++y)for(int x=0;x<61;++x)v.h[y*61+x]=tangent*(.6*(x-30)+.8*(y-30))*r+(x+y>60?.499f:0.f);
  auto m=MeasurePlatformTerrain(v,{30,30},.68);EXPECT_NEAR(m.slope_rad,19.9*std::numbers::pi/180.,1e-6);EXPECT_NEAR(m.relief_m,.499,1e-6);
 }
}
TEST(PlatformTerrainWindow, DiagonalNarrowRidgeAndPitRetainHeight){
 for(double r:{.05,.1,.2})for(float height:{.501f,-.501f}){
  View v(r);for(int y=0;y<61;++y)v.h[y*61+y]=height;
  auto m=MeasurePlatformTerrain(v,{30,30},.68);EXPECT_NEAR(m.relief_m,.501,1e-6);
 }
}
TEST(PlatformTerrainWindow, SparseNoncollinearObservedSupportIsEnoughWithoutCoverageGate){
 View v(.05);std::fill(v.h.begin(),v.h.end(),std::numeric_limits<float>::quiet_NaN());
 v.h[30*61+30]=0.f;v.h[30*61+31]=.01f;v.h[31*61+30]=.005f;
 auto m=MeasurePlatformTerrain(v,{30,30},.68);EXPECT_TRUE(m.center_known);EXPECT_TRUE(m.neighborhood_complete);
 LeggedCapability cap;cap.body_extent_m={.68,.33,.35};cap.maximum_slope_rad=.35;cap.maximum_step_height_m=.5;cap.maximum_gap_width_m=.3;cap.minimum_body_clearance_m=.3;
 auto evaluator=MakePlatformWindowElevationEvaluator(v,cap);EXPECT_EQ(evaluator->Evaluate(v,{30,30}).state,IntrinsicCellState::kFree);
}
TEST(PlatformTerrainWindow, CollinearLocalSupportIsInsufficientEvenWhenLWindowHasTwoGradients){
 View v(.05);std::fill(v.h.begin(),v.h.end(),std::numeric_limits<float>::quiet_NaN());
 for(int x=23;x<=37;++x){v.h[24*61+x]=0.f;v.h[25*61+x]=0.f;v.h[30*61+x]=0.f;}
 auto m=MeasurePlatformTerrain(v,{30,30},.68);EXPECT_TRUE(m.center_known);EXPECT_FALSE(m.neighborhood_complete);
}
TEST(PlatformTerrainWindow, IsolatedObservationDoesNotInventGround){
 View v(.05);std::fill(v.h.begin(),v.h.end(),std::numeric_limits<float>::quiet_NaN());v.h[30*61+30]=0.f;
 auto m=MeasurePlatformTerrain(v,{30,30},.68);EXPECT_TRUE(m.center_known);EXPECT_FALSE(m.neighborhood_complete);
}
TEST(PlatformTerrainWindow, MissingNeighborsDoNotEraseMeasuredPeak){
 View v(.05);v.h[30*61+30]=.501f;v.h[31*61+31]=std::numeric_limits<float>::quiet_NaN();
 LeggedCapability cap;cap.body_extent_m={.68,.33,.35};cap.maximum_slope_rad=.35;cap.maximum_step_height_m=.5;cap.maximum_gap_width_m=.3;cap.minimum_body_clearance_m=.3;
 auto evaluator=MakePlatformWindowElevationEvaluator(v,cap);EXPECT_EQ(evaluator->Evaluate(v,{30,30}).state,IntrinsicCellState::kBlocked);
}
TEST(PlatformTerrainWindow, BatchReadsEverySourceHaloCellAtMostOnce){
 View source(.05);source.Plane(.1);
 class CountingView final:public ElevationRangeView {
  const ElevationRangeView& source_;
  public:mutable std::size_t reads{};mutable std::vector<std::size_t> per_cell=std::vector<std::size_t>(61*61);
  explicit CountingView(const ElevationRangeView& source):source_(source){}
  const SparseGridGeometry& geometry()const noexcept override{return source_.geometry();}
  std::optional<ElevationRange> ElevationRangeAt(GridIndex i)const noexcept override{
   ++reads;if(geometry().Contains(i))++per_cell[i.y*61+i.x];return source_.ElevationRangeAt(i);
  }
 } counted(source);
 auto batch=MeasurePlatformTerrainBatch(counted,1.182,{14,14},{46,46});
 EXPECT_EQ(counted.reads,56U*56U);
 EXPECT_EQ(*std::max_element(counted.per_cell.begin(),counted.per_cell.end()),1U);
 EXPECT_TRUE(batch.At({30,30})->neighborhood_complete);
 EXPECT_NEAR(batch.At({30,30})->slope_rad,.1,1e-6);
}
TEST(PlatformTerrainWindow, ReusedMedianNodesMatchNaiveWindowWithHolesAndDuplicates){
 for(std::size_t width:{1U,2U,3U,7U,25U,129U}){
  map::SlidingTerrainMedian median;std::deque<std::optional<double>> active;
  std::uint32_t seed=0x12345678U;
  for(int sample=0;sample<2500;++sample){
   seed=seed*1664525U+1013904223U;
   std::optional<double> value=seed%7==0?std::nullopt:std::optional(double(int(seed%17)-8)/7.);
   median.Push(value);active.push_back(value);
   if(active.size()>width){median.Pop();active.pop_front();}
   std::vector<double> sorted;for(auto x:active)if(x)sorted.push_back(*x);
   std::sort(sorted.begin(),sorted.end());
   auto actual=median.Median();
   if(sorted.empty()){EXPECT_FALSE(actual);continue;}
   ASSERT_TRUE(actual);
   auto middle=sorted.size()/2;
   double expected=sorted.size()%2?sorted[middle]:.5*sorted[middle-1]+.5*sorted[middle];
   EXPECT_EQ(*actual,expected)<<"width="<<width<<" sample="<<sample;
  }
  while(!active.empty()){median.Pop();active.pop_front();}
  EXPECT_FALSE(median.Median());
 }
}
} }
