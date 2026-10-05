#include <gtest/gtest.h>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "map/terrain_sample_filter.hpp"

using namespace lunar::incremental_navigation;
using namespace lunar::incremental_navigation::map;
namespace {
SurfaceSample Point(double x, double y, double z) { return {x,y,z,z,z}; }
void ExpectRawMetadata(const std::vector<SurfaceSample>& raw,
                       const std::vector<SurfaceSample>& filtered) {
  ASSERT_EQ(raw.size(),filtered.size());
  for(std::size_t i=0;i<raw.size();++i) {
    EXPECT_DOUBLE_EQ(raw[i].x,filtered[i].x);
    EXPECT_DOUBLE_EQ(raw[i].y,filtered[i].y);
    EXPECT_DOUBLE_EQ(raw[i].low,filtered[i].low);
    EXPECT_DOUBLE_EQ(raw[i].high,filtered[i].high);
  }
}
TEST(TerrainSampleFilter, MedianRemovesIsolatedNoiseButRetainsRawHeightIntervals) {
  const std::vector<SurfaceSample> raw{Point(0,0,.025),Point(.05,0,0),
      Point(-.05,0,0),Point(0,.05,0),Point(0,-.05,0)};
  const auto filtered=FilterTerrainSamples(raw,TerrainFilterMethod::kMedian);
  ExpectRawMetadata(raw,filtered);
  for(const auto& p:filtered) EXPECT_DOUBLE_EQ(p.z,0.);
  EXPECT_DOUBLE_EQ(raw[0].z,.025);
  EXPECT_DOUBLE_EQ(filtered[0].high,.025);
}
TEST(TerrainSampleFilter, EvenNeighbourCountUsesMeanOfMiddleHeights) {
  const std::vector<SurfaceSample> raw{Point(0,0,-.02),Point(.02,0,0),
      Point(0,.02,.02),Point(.02,.02,.08)};
  const auto filtered=FilterTerrainSamples(raw,TerrainFilterMethod::kMedian);
  for(const auto& p:filtered) EXPECT_DOUBLE_EQ(p.z,.01);
}
TEST(TerrainSampleFilter, NeighbourhoodUsesPhysicalEuclideanRadius) {
  const std::vector<SurfaceSample> raw{Point(0,0,0),Point(.05,0,.01),
      Point(0,.05,.02),Point(.08,.08,.5),Point(.101,0,.9)};
  const auto filtered=FilterTerrainSamples(raw,TerrainFilterMethod::kMedian);
  // Both large values are outside the centre's 10 cm disk.
  EXPECT_DOUBLE_EQ(filtered[0].z,.01);
}
TEST(TerrainSampleFilter, SparseNeighbourhoodAndEmptyInputKeepTheirSamples) {
  const std::vector<SurfaceSample> raw{Point(0,0,.02),Point(.05,0,-.02),Point(.4,0,.1)};
  for(auto method:{TerrainFilterMethod::kMedian,TerrainFilterMethod::kBilateral}) {
    const auto filtered=FilterTerrainSamples(raw,method);
    ExpectRawMetadata(raw,filtered);
    for(std::size_t i=0;i<raw.size();++i) EXPECT_DOUBLE_EQ(filtered[i].z,raw[i].z);
    EXPECT_TRUE(FilterTerrainSamples({},method).empty());
  }
}
TEST(TerrainSampleFilter, MedianIsOnePassOverOriginalSamples) {
  const std::vector<SurfaceSample> raw{Point(0,0,0),Point(.05,0,.03),
      Point(.10,0,0),Point(.15,0,.03),Point(.20,0,0)};
  const auto filtered=FilterTerrainSamples(raw,TerrainFilterMethod::kMedian);
  const double expected[]{0.,.015,0.,.015,0.};
  for(std::size_t i=0;i<raw.size();++i) EXPECT_NEAR(filtered[i].z,expected[i],1e-14);
  auto reversed=raw; std::reverse(reversed.begin(),reversed.end());
  auto reverse_filtered=FilterTerrainSamples(reversed,TerrainFilterMethod::kMedian);
  std::reverse(reverse_filtered.begin(),reverse_filtered.end());
  for(std::size_t i=0;i<raw.size();++i) EXPECT_DOUBLE_EQ(filtered[i].z,reverse_filtered[i].z);
}
TEST(TerrainSampleFilter, BilateralReducesSmallNoiseAndPreservesSharpHeightContrast) {
  const std::vector<SurfaceSample> noisy{Point(0,0,.015),Point(.05,0,0),
      Point(-.05,0,0),Point(0,.05,0),Point(0,-.05,0)};
  const auto filtered=FilterTerrainSamples(noisy,TerrainFilterMethod::kBilateral);
  ExpectRawMetadata(noisy,filtered);
  EXPECT_GT(filtered[0].z,0.);
  EXPECT_LT(filtered[0].z,.006);
  auto sharp=noisy; sharp[0]=Point(0,0,.2);
  const auto edge=FilterTerrainSamples(sharp,TerrainFilterMethod::kBilateral);
  EXPECT_NEAR(edge[0].z,.2,1e-8);
  EXPECT_DOUBLE_EQ(sharp[0].z,.2);
}
TEST(TerrainSampleFilter, BilateralOnePassIsIndependentOfSampleOrder) {
  const std::vector<SurfaceSample> raw{Point(0,0,0),Point(.05,0,.03),
      Point(.10,0,-.01),Point(.15,0,.02),Point(.20,0,0)};
  const auto filtered=FilterTerrainSamples(raw,TerrainFilterMethod::kBilateral);
  auto reversed=raw; std::reverse(reversed.begin(),reversed.end());
  auto reverse_filtered=FilterTerrainSamples(reversed,TerrainFilterMethod::kBilateral);
  std::reverse(reverse_filtered.begin(),reverse_filtered.end());
  for(std::size_t i=0;i<raw.size();++i) EXPECT_NEAR(filtered[i].z,reverse_filtered[i].z,1e-14);
  EXPECT_LT(filtered[1].z,raw[1].z);
}
TEST(TerrainSampleFilter, NonePreservesEveryDoubleBitAndAddsNoSamples) {
  const std::vector<SurfaceSample> raw{Point(-0.,0,.025),Point(.05,0,-0.),
      Point(1,1,std::numeric_limits<double>::quiet_NaN())};
  const auto filtered=FilterTerrainSamples(raw,TerrainFilterMethod::kNone);
  ASSERT_EQ(raw.size(),filtered.size());
  for(std::size_t i=0;i<raw.size();++i)
    for(auto values:{std::pair{raw[i].x,filtered[i].x},std::pair{raw[i].y,filtered[i].y},
        std::pair{raw[i].low,filtered[i].low},std::pair{raw[i].high,filtered[i].high},
        std::pair{raw[i].z,filtered[i].z}})
      EXPECT_EQ(std::bit_cast<std::uint64_t>(values.first),std::bit_cast<std::uint64_t>(values.second));
}
TEST(TerrainSampleFilter, InvalidMethodAndInvalidFilteredSamplesAreRejected) {
  const std::vector<SurfaceSample> raw{Point(0,0,0)};
  const auto invalid=static_cast<TerrainFilterMethod>(255);
  EXPECT_EQ(TerrainFilterMethodName(invalid),"invalid");
  EXPECT_THROW(FilterTerrainSamples(raw,invalid),std::invalid_argument);
  EXPECT_THROW(ParseTerrainFilterMethod("unknown"),std::invalid_argument);
  for(auto method:{TerrainFilterMethod::kNone,TerrainFilterMethod::kMedian,TerrainFilterMethod::kBilateral})
    EXPECT_EQ(ParseTerrainFilterMethod(TerrainFilterMethodName(method)),method);
  auto bad=raw; bad[0].z=std::numeric_limits<double>::infinity();
  EXPECT_THROW(FilterTerrainSamples(bad,TerrainFilterMethod::kMedian),std::invalid_argument);
  EXPECT_THROW(FilterTerrainSamples(bad,TerrainFilterMethod::kBilateral),std::invalid_argument);
}
}  // namespace
