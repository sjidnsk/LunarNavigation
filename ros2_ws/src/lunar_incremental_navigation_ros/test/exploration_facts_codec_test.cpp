#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <string>
#include "lunar_incremental_navigation_ros/exploration_facts_codec.hpp"

namespace lunar::incremental_navigation_ros {
namespace core = lunar::incremental_navigation;
namespace {
// 以下JSON由Python encode_observation_facts真实编码生成，保留跨语言数值合同。
std::string Fixture(const std::string& name) {
  if (name == "source05") return R"facts({"schema":"lunar-observation-facts/v1","source_resolution_m":0.05,"raw_revision":9223372036854775931,"frame":"map","epoch":7,"complete_snapshot":true,"observation_fragments":[{"polygon":[[-0.05,0.0],[0.0,0.05],[0.05,0.0],[0.0,-0.05]],"height_m":1.0,"slope_rad":null,"relief_m":null,"state":-1},{"polygon":[[0.15000000000000002,0.0],[0.2,0.0],[0.2,0.05],[0.15000000000000002,0.05]],"height_m":2.0,"slope_rad":0.2,"relief_m":0.3,"state":0},{"polygon":[[0.25,0.0],[0.30000000000000004,0.0],[0.30000000000000004,0.05],[0.25,0.05]],"height_m":3.0,"slope_rad":0.4,"relief_m":0.5,"state":1}],"processed_stamp_ns":2147483647999999999})facts";
  if (name == "source20") return R"facts({"schema":"lunar-observation-facts/v1","source_resolution_m":0.2,"raw_revision":9223372036854775931,"frame":"map","epoch":7,"complete_snapshot":true,"observation_fragments":[{"polygon":[[-0.2,0.0],[0.0,0.2],[0.2,0.0],[0.0,-0.2]],"height_m":1.0,"slope_rad":null,"relief_m":null,"state":-1},{"polygon":[[0.6000000000000001,0.0],[0.8,0.0],[0.8,0.2],[0.6000000000000001,0.2]],"height_m":2.0,"slope_rad":0.2,"relief_m":0.3,"state":0},{"polygon":[[1.0,0.0],[1.2000000000000002,0.0],[1.2000000000000002,0.2],[1.0,0.2]],"height_m":3.0,"slope_rad":0.4,"relief_m":0.5,"state":1}],"processed_stamp_ns":2147483647999999999})facts";
  if (name == "empty") return R"facts({"schema":"lunar-observation-facts/v1","source_resolution_m":0.05,"raw_revision":0,"frame":"map","epoch":8,"complete_snapshot":true,"observation_fragments":[],"processed_stamp_ns":0})facts";
  throw std::runtime_error("unknown fixture");
}
core::WheeledCapability Wheel() {
  return {.footprint_xy_m = {{-.01, -.01}, {.01, -.01}, {.01, .01}, {-.01, .01}},
      .body_extent_m = {.02, .02, .02}, .minimum_underbody_clearance_m = .1,
      .maximum_local_obstacle_relief_m = .2, .maximum_slope_rad = .5,
      .minimum_clearance_m = 0.};
}
TEST(ExplorationFactsCodec, PythonSnapshotsPreserveAllObservationValues) {
  for (const auto& name : {"source05", "source20"}) {
    const auto [facts, stamp] = ParseExplorationFacts(Fixture(name));
    const double r = std::string(name) == "source05" ? .05 : .2;
    EXPECT_DOUBLE_EQ(facts.source_resolution_m, r);
    EXPECT_EQ(facts.raw_revision, UINT64_C(9223372036854775931));
    EXPECT_EQ(stamp, INT64_C(2147483647999999999));
    EXPECT_EQ(facts.frame, "map");
    EXPECT_EQ(facts.epoch, "7");
    EXPECT_TRUE(facts.complete_snapshot);
    ASSERT_EQ(facts.observation_fragments.size(), 3U);
    const auto& unknown = facts.observation_fragments[0];
    EXPECT_EQ(unknown.state, core::IntrinsicCellState::kUnknown);
    EXPECT_TRUE(std::isnan(unknown.slope_rad));
    EXPECT_TRUE(std::isnan(unknown.relief_m));
    EXPECT_FLOAT_EQ(unknown.height_m, 1.F);
    ASSERT_EQ(unknown.polygon.size(), 4U);
    EXPECT_DOUBLE_EQ(unknown.polygon[0].x, -r);
    EXPECT_DOUBLE_EQ(unknown.polygon[1].y, r);
    EXPECT_EQ(facts.observation_fragments[1].state, core::IntrinsicCellState::kFree);
    EXPECT_EQ(facts.observation_fragments[2].state, core::IntrinsicCellState::kBlocked);
    EXPECT_DOUBLE_EQ(facts.observation_fragments[1].slope_rad, .2);
  }
}
TEST(ExplorationFactsCodec, EmptyCompleteSnapshotAndDefaultStamp) {
  const auto [facts, stamp] = ParseExplorationFacts(Fixture("empty"));
  EXPECT_TRUE(facts.complete_snapshot);
  EXPECT_TRUE(facts.observation_fragments.empty());
  EXPECT_EQ(facts.raw_revision, 0U);
  EXPECT_EQ(stamp, 0);
}
TEST(ExplorationFactsCodec, ParsedSnapshotFeedsApplyFactsWithoutReclassification) {
  const auto [facts, stamp] = ParseExplorationFacts(Fixture("source05"));
  core::TraversabilityProfile profile{.planar_envelope_xy_m = Wheel().footprint_xy_m,
      .slope_weight = 1., .relief_weight = 1., .goal_position_tolerance_m = .1,
      .goal_yaw_tolerance_rad = .05};
  ExplorationMap map(Wheel(), profile);
  ASSERT_TRUE(map.ApplyFacts(facts, stamp));
  EXPECT_EQ(map.SourceEpoch(), "7");
  EXPECT_EQ(map.SourceRawRevision(), facts.raw_revision);
  EXPECT_EQ(map.Capture().map.processed_stamp_ns, stamp);
  const auto [empty, empty_stamp] = ParseExplorationFacts(Fixture("empty"));
  EXPECT_TRUE(map.ApplyFacts(empty, empty_stamp));
  EXPECT_EQ(map.SourceRawRevision(), 0U);
}
TEST(ExplorationFactsCodec, RejectsIncompleteOrMalformedProtocol) {
  const std::string text = Fixture("source05");
  for (const auto key : {"schema", "source_resolution_m", "raw_revision", "frame", "epoch",
                         "complete_snapshot", "processed_stamp_ns", "observation_fragments"}) {
    auto node = YAML::Load(text);
    node.remove(key);
    EXPECT_THROW((void)ParseExplorationFacts(YAML::Dump(node)), std::invalid_argument);
  }
  for (const auto field : {"polygon", "height_m", "slope_rad", "relief_m", "state"}) {
    auto node = YAML::Load(text);
    node["observation_fragments"][0].remove(field);
    EXPECT_THROW((void)ParseExplorationFacts(YAML::Dump(node)), std::invalid_argument);
  }
  for (const auto& change : {
      std::pair{"schema", "v2"}, {"source_resolution_m", "0"},
      {"source_resolution_m", ".inf"}, {"raw_revision", "-1"},
      {"raw_revision", "18446744073709551616"}, {"epoch", "-1"},
      {"processed_stamp_ns", "2147483648000000000"},
      {"processed_stamp_ns", "9223372036854775791"},
      {"processed_stamp_ns", "9223372036854775808"}, {"processed_stamp_ns", "-1"},
      {"complete_snapshot", "false"}}) {
    auto node = YAML::Load(text);
    node[change.first] = YAML::Load(change.second);
    EXPECT_THROW((void)ParseExplorationFacts(YAML::Dump(node)), std::invalid_argument);
  }
  for (const auto& change : {std::pair{"state", "2"}, {"state", "0.0"},
      {"state", "true"}, {"height_m", ".nan"}, {"height_m", "1e100"},
      {"slope_rad", "-.1"}, {"relief_m", ".inf"}, {"polygon", "[[0,0],[1,1]]"},
      {"polygon", "[[0,0],[1,1],[2,3,4]]"}}) {
    auto node = YAML::Load(text);
    node["observation_fragments"][0][change.first] = YAML::Load(change.second);
    EXPECT_THROW((void)ParseExplorationFacts(YAML::Dump(node)), std::invalid_argument);
  }
  auto unknown = YAML::Load(text);
  unknown["source_id"] = 1;
  EXPECT_THROW((void)ParseExplorationFacts(YAML::Dump(unknown)), std::invalid_argument);
  EXPECT_THROW((void)ParseExplorationFacts(text.substr(0, text.size() - 1) + ", \"epoch\":8}"),
               std::invalid_argument);
}
TEST(ExplorationFactsCodec, RejectsJsonScalarTypeConfusionAndNestedDuplicateKeys) {
  const std::string text = Fixture("source05");
  for (const auto& replacement : {
      std::pair{R"("frame":"map")", R"("frame":123)"},
      {R"("frame":"map")", R"("frame":true)"},
      {R"("epoch":7)", R"("epoch":"7")"},
      {R"("processed_stamp_ns":2147483647999999999)", R"("processed_stamp_ns":true)"},
      {R"("height_m":1.0)", R"("height_m":"1.0")"},
      {R"("height_m":1.0)", R"("height_m":1.0,"height_m":1.0)"},
      {R"("state":-1)", R"("state":false)"}}) {
    auto changed = text;
    const auto position = changed.find(replacement.first);
    ASSERT_NE(position, std::string::npos);
    changed.replace(position, std::string(replacement.first).size(), replacement.second);
    EXPECT_THROW((void)ParseExplorationFacts(changed), std::invalid_argument);
  }
}
}  // namespace
}  // namespace lunar::incremental_navigation_ros
