cmake_minimum_required(VERSION 3.22)
project(lunar_pure_wheeled_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(lunar_planning_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME})

install(PROGRAMS
  scripts/lunar_pure_wheeled_controller_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/orin_controller.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/wheel_orin.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/orin_legged_controller.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/legged_orin.yaml"
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_deployment_policy_test test/test_deployment_policy.py)
  ament_add_pytest_test(${PROJECT_NAME}_deployment_launch_test test/test_deployment_launch.py)
  ament_add_pytest_test(${PROJECT_NAME}_platform_policy_test test/test_platform_policy.py)
  ament_add_pytest_test(${PROJECT_NAME}_legged_closed_loop_test test/test_legged_closed_loop.py TIMEOUT 90)
  ament_add_pytest_test(${PROJECT_NAME}_shutdown_test test/test_shutdown.py)
  ament_add_pytest_test(${PROJECT_NAME}_reference_test test/test_reference.py)
  ament_add_pytest_test(${PROJECT_NAME}_incremental_path_test test/test_incremental_path.py)
  ament_add_pytest_test(${PROJECT_NAME}_tracking_test test/test_tracking.py)
  ament_add_pytest_test(${PROJECT_NAME}_execution_test test/test_execution.py)
  ament_add_pytest_test(${PROJECT_NAME}_node_test test/test_node.py)
endif()

ament_package()
