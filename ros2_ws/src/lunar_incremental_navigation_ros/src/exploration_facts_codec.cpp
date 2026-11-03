#include "lunar_incremental_navigation_ros/exploration_facts_codec.hpp"

#include <yaml-cpp/yaml.h>

#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace lunar::incremental_navigation_ros {
namespace {
namespace core = lunar::incremental_navigation;

void RequireFields(const YAML::Node& node,
                   const std::set<std::string>& expected) {
  if (!node.IsMap()) throw std::invalid_argument("facts must contain objects");
  std::set<std::string> found;
  for (const auto& item : node) {
    if (!item.first.IsScalar()) throw std::invalid_argument("invalid facts field");
    const std::string key = item.first.Scalar();
    if (!expected.contains(key) || !found.insert(key).second) {
      throw std::invalid_argument("unknown or duplicate facts field: " + key);
    }
  }
  if (found != expected) throw std::invalid_argument("incomplete facts fields");
}

// 引号内数字不是JSON数字；from_chars保留64位整数，不经double中转。
void RequireNumericScalar(const YAML::Node& node) {
  if (!node.IsScalar() || node.Tag() == "!") {
    throw std::invalid_argument("facts require numeric scalars");
  }
}
template <typename Integer>
Integer IntegerValue(const YAML::Node& node) {
  RequireNumericScalar(node);
  const std::string& text = node.Scalar();
  Integer result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    throw std::invalid_argument("facts require an in-range decimal integer");
  }
  return result;
}
double FiniteNumber(const YAML::Node& node) {
  RequireNumericScalar(node);
  const double value = node.as<double>();
  if (!std::isfinite(value)) throw std::invalid_argument("facts require finite numbers");
  return value;
}
double OptionalMeasurement(const YAML::Node& node) {
  if (node.IsNull()) return std::numeric_limits<double>::quiet_NaN();
  const double value = FiniteNumber(node);
  if (value < 0.) throw std::invalid_argument("negative physical measurement");
  return value;
}
}  // namespace

std::pair<ExplorationMapFacts, std::int64_t> ParseExplorationFacts(
    const std::string& payload) {
  try {
    const YAML::Node root = YAML::Load(payload);
    RequireFields(root, {"schema", "source_resolution_m", "raw_revision", "frame",
                        "epoch", "complete_snapshot", "processed_stamp_ns",
                        "observation_fragments"});
    if (!root["schema"].IsScalar() ||
        root["schema"].Scalar() != "lunar-observation-facts/v1" ||
        !root["complete_snapshot"].IsScalar() ||
        root["complete_snapshot"].Tag() == "!" ||
        root["complete_snapshot"].Scalar() != "true") {
      throw std::invalid_argument("unsupported or incomplete observation snapshot");
    }
    ExplorationMapFacts facts;
    facts.complete_snapshot = true;
    facts.source_resolution_m = FiniteNumber(root["source_resolution_m"]);
    if (facts.source_resolution_m <= 0.) {
      throw std::invalid_argument("source resolution must be positive");
    }
    facts.raw_revision = IntegerValue<std::uint64_t>(root["raw_revision"]);
    // Python会话epoch为非负整数；派生接口使用其十进制字符串，不使用native handle。
    facts.epoch = std::to_string(IntegerValue<std::uint64_t>(root["epoch"]));
    if (!root["frame"].IsScalar() || root["frame"].Tag() != "!") {
      throw std::invalid_argument("frame must be a JSON string");
    }
    facts.frame = root["frame"].Scalar();
    if (facts.frame.find_first_not_of(" \t\r\n") == std::string::npos) {
      throw std::invalid_argument("frame must be nonempty");
    }
    const auto stamp = IntegerValue<std::int64_t>(root["processed_stamp_ns"]);
    if (stamp < 0 || stamp > INT64_C(2147483647999999999))
      throw std::invalid_argument("processed stamp must fit nonnegative ROS Time");
    const YAML::Node fragments = root["observation_fragments"];
    if (!fragments.IsSequence()) throw std::invalid_argument("fragments must be an array");
    for (const auto& node : fragments) {
      RequireFields(node, {"polygon", "height_m", "slope_rad", "relief_m", "state"});
      ExplorationObservationFragment fragment;
      const double height = FiniteNumber(node["height_m"]);
      if (std::abs(height) > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("height exceeds finite storage range");
      }
      fragment.height_m = static_cast<float>(height);
      fragment.slope_rad = OptionalMeasurement(node["slope_rad"]);
      fragment.relief_m = OptionalMeasurement(node["relief_m"]);
      switch (IntegerValue<int>(node["state"])) {
        case -1: fragment.state = core::IntrinsicCellState::kUnknown; break;
        case 0: fragment.state = core::IntrinsicCellState::kFree; break;
        case 1: fragment.state = core::IntrinsicCellState::kBlocked; break;
        default: throw std::invalid_argument("invalid intrinsic observation state");
      }
      const YAML::Node polygon = node["polygon"];
      if (!polygon.IsSequence() || polygon.size() < 3U) {
        throw std::invalid_argument("polygon requires at least three points");
      }
      for (const auto& point : polygon) {
        if (!point.IsSequence() || point.size() != 2U) {
          throw std::invalid_argument("polygon point must have two coordinates");
        }
        fragment.polygon.push_back({FiniteNumber(point[0]), FiniteNumber(point[1])});
      }
      facts.observation_fragments.push_back(std::move(fragment));
    }
    return {std::move(facts), stamp};
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument(std::string("invalid observation facts: ") + error.what());
  }
}
}  // namespace lunar::incremental_navigation_ros
