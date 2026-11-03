#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "lunar_incremental_navigation_ros/exploration_map.hpp"

namespace lunar::incremental_navigation_ros {
// 解析完整观测协议。Python状态 -1/0/1 显式映射到本征状态枚举。
// 非法字段、版本、数字或不完整快照抛出 std::invalid_argument。
[[nodiscard]] std::pair<ExplorationMapFacts, std::int64_t> ParseExplorationFacts(
    const std::string& payload);
}  // namespace lunar::incremental_navigation_ros
