#include "lunar_incremental_navigation_ros/platform_config.hpp"

#include <filesystem>
#include <fstream>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

TEST(PlatformConfig, OrinLeggedProfileLoadsLeggedCapabilityAndRejectsWheelSelector) {
  const auto path = std::string{PLATFORM_CONFIG_DIR} + "/legged_orin.yaml";
  const auto result = lunar::incremental_navigation_ros::LoadPlatformConfig(path, "legged");
  ASSERT_TRUE(result.capability) << result.error_detail;
  const auto& legged = std::get<lunar::incremental_navigation::LeggedCapability>(*result.capability);
  EXPECT_DOUBLE_EQ(legged.maximum_step_height_m, 0.5);
  EXPECT_DOUBLE_EQ(legged.maximum_gap_width_m, 0.3);
  EXPECT_FALSE(legged.unknown_is_traversable);
  ASSERT_TRUE(result.start_blind_zone_margin_m);
  EXPECT_DOUBLE_EQ(*result.start_blind_zone_margin_m, 3.0);
  EXPECT_FALSE(lunar::incremental_navigation_ros::LoadPlatformConfig(path, "wheel").capability);
}

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

TEST(PlatformConfig, MissingMarginUsesDeploymentDefaultAndExplicitZeroWins) {
  for (const std::string platform : {"wheel", "legged"}) {
    const auto generic = std::string{PLATFORM_CONFIG_DIR} + "/" + platform + ".yaml";
    const auto defaults = lunar::incremental_navigation_ros::LoadPlatformConfig(generic, platform);
    ASSERT_TRUE(defaults.capability) << defaults.error_detail;
    ASSERT_TRUE(defaults.start_blind_zone_margin_m);
    EXPECT_DOUBLE_EQ(*defaults.start_blind_zone_margin_m, 3.0);
    auto config = YAML::LoadFile(generic);
    config["start_blind_zone_margin_m"] = 0.0;
    const auto path = std::filesystem::temp_directory_path() / ("incremental_zero_margin_" + platform + ".yaml");
    { std::ofstream output(path); output << config; }
    const auto explicit_zero = lunar::incremental_navigation_ros::LoadPlatformConfig(path, platform);
    ASSERT_TRUE(explicit_zero.capability) << explicit_zero.error_detail;
    ASSERT_TRUE(explicit_zero.start_blind_zone_margin_m);
    EXPECT_DOUBLE_EQ(*explicit_zero.start_blind_zone_margin_m, 0.0);
    std::filesystem::remove(path);
  }
}
