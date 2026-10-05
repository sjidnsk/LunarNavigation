#include <cmath>
#include <gtest/gtest.h>
#include "map/terrain_surface_fit.hpp"
using namespace lunar::incremental_navigation::map;
namespace {
std::vector<SurfaceSample> Plane() {
  std::vector<SurfaceSample> samples;
  for (int y=-4;y<=4;++y) for(int x=-4;x<=4;++x) {
    const double xx=x*.05, yy=y*.05, z=1+.2*xx-.1*yy;
    samples.push_back({xx,yy,z,z,z});
  }
  return samples;
}
}
TEST(TerrainGaussianProcess, PreservesPlanarMeanAndBoundsSampleBudget) {
  auto fit=FitGaussianProcess(Plane(),{});
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->Height(0,0),1,1e-9);
  EXPECT_NEAR(fit->Gradient(0,0).x,.2,1e-8);
  EXPECT_NEAR(fit->Gradient(0,0).y,-.1,1e-8);
  EXPECT_LE(fit->gp_samples.size(),32u);
  EXPECT_GE(fit->predictive_variance_m2,0);
  EXPECT_TRUE(std::isfinite(fit->predictive_variance_m2));
}
TEST(TerrainGaussianProcess, FitsSmoothResidualAndIsDeterministic) {
  auto samples=Plane();
  for(auto& p:samples) p.low=p.high=p.z=1+.8*(p.x*p.x+p.y*p.y);
  auto a=FitGaussianProcess(samples,{}), b=FitGaussianProcess(samples,{});
  ASSERT_TRUE(a); ASSERT_TRUE(b);
  EXPECT_LT(std::abs(a->Height(0,0)-1),.012);
  EXPECT_DOUBLE_EQ(a->Height(0,0),b->Height(0,0));
}
TEST(TerrainGaussianProcess, RejectsCollinearSupport) {
  std::vector<SurfaceSample> points;
  for(int x=-4;x<=4;++x) points.push_back({x*.05,0,0,0,0});
  EXPECT_FALSE(FitGaussianProcess(points,{}));
}
TEST(TerrainGaussianProcess, DoesNotLearnLargeIsolatedOutlier) {
  auto samples=Plane(); samples.front().low=samples.front().high=samples.front().z=2;
  auto fit=FitGaussianProcess(samples,{});
  ASSERT_TRUE(fit); EXPECT_NEAR(fit->Height(0,0),1,.003);
  EXPECT_FALSE(fit->inliers.front());
}
