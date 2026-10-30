#include "lunar_incremental_navigation_ros/platform_config.hpp"

#include <filesystem>
#include <fstream>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

TEST(PlatformConfig, ModifiedProfileUsesConfiguredValuesAndRejectsInvalidSpeed) {
  auto config = YAML::LoadFile(std::string{PLATFORM_CONFIG_DIR} + "/wheel.yaml");
  config["capability"]["maximum_forward_speed_mps"] = 0.1;
  config["capability"]["maximum_acceleration_mps2"] = 0.25;
  config["capability_version"] = "custom-profile";
  auto primitives = config["capability"]["motion_primitives"];
  primitives.remove(primitives.size() - 1U);
  const auto path = std::filesystem::temp_directory_path() / "incremental_flexible_config_test.yaml";
  const auto write = [&] { std::ofstream stream(path); stream << config; };
  write();
  const auto result = lunar::incremental_navigation_ros::LoadPlatformConfig(path, "wheel");
  ASSERT_TRUE(result.capability) << result.error_detail;
  const auto& wheel = std::get<lunar::incremental_navigation::WheeledCapability>(*result.capability);
  EXPECT_DOUBLE_EQ(wheel.maximum_forward_speed_mps, 0.1);
  EXPECT_DOUBLE_EQ(wheel.maximum_acceleration_mps2, 0.25);
  EXPECT_EQ(wheel.motion_primitives.size(), 8U);
  config["capability"]["maximum_forward_speed_mps"] = -0.1;
  write();
  const auto invalid = lunar::incremental_navigation_ros::LoadPlatformConfig(path, "wheel");
  EXPECT_FALSE(invalid.capability);
  EXPECT_NE(invalid.error_detail.find("maximum_forward_speed_mps"), std::string::npos);
  std::filesystem::remove(path);
}

// 演示能力档也必须走真实严格parser；simulation_only仅属于Python profile。
TEST(PlatformConfig, IsaacNavigationDemoUsesStrictSchemaAndRetainsSafety) {
  const auto path = std::filesystem::path{PLATFORM_CONFIG_DIR} /
                    "wheel_isaac_navigation_demo.yaml";
  const auto result = lunar::incremental_navigation_ros::LoadPlatformConfig(path, "wheel");
  ASSERT_TRUE(result.capability) << result.error_detail;
  const auto& wheel = std::get<lunar::incremental_navigation::WheeledCapability>(*result.capability);
  EXPECT_DOUBLE_EQ(wheel.maximum_forward_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(wheel.maximum_spin_rate_radps, 2.0);
  EXPECT_DOUBLE_EQ(wheel.maximum_reverse_speed_mps, 0.2);
  EXPECT_DOUBLE_EQ(wheel.maximum_acceleration_mps2, 0.5);
  EXPECT_DOUBLE_EQ(wheel.maximum_braking_deceleration_mps2, 0.5);
  EXPECT_DOUBLE_EQ(wheel.maximum_yaw_acceleration_radps2, 0.5);
  EXPECT_DOUBLE_EQ(wheel.maximum_lateral_acceleration_mps2, 0.5);
  EXPECT_DOUBLE_EQ(wheel.maximum_slope_rad, 0.3490658503988659);
  EXPECT_DOUBLE_EQ(wheel.minimum_clearance_m, 0.4);
  EXPECT_EQ(wheel.footprint_xy_m.size(), 4U);
  EXPECT_EQ(wheel.motion_primitives.size(), 9U);

  auto invalid = YAML::LoadFile(path.string());
  invalid["simulation_only"] = true;
  const auto invalid_path = std::filesystem::temp_directory_path() /
                            "incremental_isaac_annotation_schema_test.yaml";
  { std::ofstream stream(invalid_path); stream << invalid; }
  const auto rejected = lunar::incremental_navigation_ros::LoadPlatformConfig(invalid_path, "wheel");
  EXPECT_FALSE(rejected.capability);
  EXPECT_NE(rejected.error_detail.find("unknown field: simulation_only"), std::string::npos);
  std::filesystem::remove(invalid_path);
}

// 使用实际配置和真实解析入口，覆盖可选字段与严格拒绝边界。
TEST(PlatformConfig, WheelMotionModelCompatibilityAndStrictValidation) {
  for (const auto* file : {"wheel.yaml", "wheel_orin.yaml"}) {
    SCOPED_TRACE(file);
    const auto original = YAML::LoadFile(std::string{PLATFORM_CONFIG_DIR} + "/" + file);
    const auto path = std::filesystem::temp_directory_path() /
        "lunar_incremental_navigation_ros_motion_model_test.yaml";
    const auto load = [&](const YAML::Node& config) {
      { std::ofstream stream(path); stream << config; }
      return lunar::incremental_navigation_ros::LoadPlatformConfig(path, "wheel");
    };
    for (const auto* model : {"differential_drive", "curvature_limited"}) {
      SCOPED_TRACE(model);
      auto config = YAML::Clone(original);
      config["capability"]["motion_model"] = model;
      const auto result = load(config);
      ASSERT_TRUE(result.capability) << result.error_detail;
      const auto& wheel = std::get<lunar::incremental_navigation::WheeledCapability>(*result.capability);
      EXPECT_DOUBLE_EQ(wheel.maximum_forward_speed_mps,
                       original["capability"]["maximum_forward_speed_mps"].as<double>());
      EXPECT_DOUBLE_EQ(wheel.footprint_xy_m.front().x,
                       original["capability"]["footprint_xy_m"][0][0].as<double>());
    }
    auto legacy = YAML::Clone(original);
    legacy["capability"].remove("motion_model");
    ASSERT_TRUE(load(legacy).capability);
    for (const auto* invalid : {"differential_driv", "", "null", "[differential_drive]", "{value: differential_drive}"}) {
      SCOPED_TRACE(invalid);
      auto config = YAML::Clone(original);
      config["capability"]["motion_model"] = YAML::Load(invalid);
      EXPECT_FALSE(load(config).capability);
    }
    auto typo = YAML::Clone(legacy);
    typo["capability"]["motion_modle"] = "differential_drive";
    EXPECT_FALSE(load(typo).capability);
    std::filesystem::remove(path);
  }
}
