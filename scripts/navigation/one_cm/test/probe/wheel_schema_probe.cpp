// 无 ROS 节点、无 gtest：链接已安装的实际 C++ loader，保留输入夹具便于审计。
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "lunar_incremental_navigation_ros/platform_config.hpp"
#include "lunar_pure_planner_ros/platform_config.hpp"
#include "lunar_pure_exploration_ros/platform_config_loader.hpp"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: wheel_schema_probe INPUT_WHEEL_YAML EXISTING_OUTPUT_DIR\n";
    return 2;
  }
  const auto original = YAML::LoadFile(argv[1]);
  const std::filesystem::path output_dir{argv[2]};
  if (!std::filesystem::is_directory(output_dir)) return 2;
  int failures = 0;
  const auto check = [&](const char* name, const YAML::Node& config, bool expected) {
    const auto path = output_dir / (std::string{name} + ".yaml");
    { std::ofstream stream(path); if (!stream) throw std::runtime_error("fixture write failed"); stream << config; }
    const auto report = [&](const char* loader, bool accepted, const std::string& detail) {
      const bool ok = accepted == expected;
      if (!ok) ++failures;
      std::cout << (ok ? "PASS " : "FAIL ") << loader << " " << name
                << " accepted=" << accepted << " expected=" << expected
                << " detail=" << detail << '\n';
    };
    const auto map = lunar::incremental_navigation_ros::LoadPlatformConfig(path, "wheel");
    report("map", map.capability.has_value(), map.error_detail);
    const auto planner = lunar::pure_planner_ros::LoadPlatformConfig(path, "wheel");
    report("planner", planner.capability.has_value(), planner.error_detail);
    for (bool incremental : {false, true}) {
      bool accepted = false;
      std::string detail;
      try {
        const auto value = incremental
            ? lunar::pure_exploration_ros::LoadIncrementalPlatformConfig(path, "wheel")
            : lunar::pure_exploration_ros::LoadPlatformConfig(path, "wheel");
        accepted = true;
        if (value.geometry.footprint_vertices.front().x !=
            original["capability"]["footprint_xy_m"][0][0].as<double>()) {
          throw std::runtime_error("footprint changed");
        }
      } catch (const std::exception& error) { detail = error.what(); accepted = false; }
      report(incremental ? "exploration_incremental" : "exploration", accepted, detail);
    }
  };
  check("actual", original, true);
  for (const auto* model : {"differential_drive", "curvature_limited"}) {
    auto config = YAML::Clone(original);
    config["capability"]["motion_model"] = model;
    check(model, config, true);
  }
  auto legacy = YAML::Clone(original);
  legacy["capability"].remove("motion_model");
  check("legacy", legacy, true);
  int index = 0;
  for (const auto* invalid : {"differential_driv", "", "null", "[differential_drive]", "{value: differential_drive}"}) {
    auto config = YAML::Clone(original);
    config["capability"]["motion_model"] = YAML::Load(invalid);
    const auto name = "invalid_" + std::to_string(index++);
    check(name.c_str(), config, false);
  }
  auto typo = YAML::Clone(legacy);
  typo["capability"]["motion_modle"] = "differential_drive";
  check("typo", typo, false);
  auto duplicate = YAML::Clone(original);
  duplicate["capability"].force_insert("motion_model", "invalid");
  check("duplicate_model", duplicate, false);
  std::cout << "failures=" << failures << '\n';
  return failures == 0 ? 0 : 1;
}
