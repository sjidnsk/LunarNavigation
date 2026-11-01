#include <gtest/gtest.h>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include "lunar_navigation_rviz/exploration_panel.hpp"
TEST(ExplorationPanel, StartsDisconnectedAndCannotSendCommandsWithoutRegionOrNode) {
  int argc = 1; char name[] = "panel_test"; char * argv[] = {name, nullptr};
  QApplication app(argc, argv);
  lunar_navigation_rviz::ExplorationPanel panel;
  EXPECT_FALSE(panel.findChild<QPushButton *>("start")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("pause")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("resume")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("cancel")->isEnabled());
  EXPECT_TRUE(panel.findChild<QLabel *>("state")->text().contains("尚未收到"));
  panel.findChild<QPushButton *>("clear")->click();
  EXPECT_FALSE(panel.findChild<QPushButton *>("start")->isEnabled());
}
