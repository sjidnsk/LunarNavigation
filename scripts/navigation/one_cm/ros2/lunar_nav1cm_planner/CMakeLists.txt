cmake_minimum_required(VERSION 3.13)
project(lunar_nav1cm_planner LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lunar_incremental_navigation_core REQUIRED)

get_filename_component(NAV1CM_CORE "${CMAKE_CURRENT_SOURCE_DIR}/../../nav1cm" ABSOLUTE)
ament_python_install_package(nav1cm PACKAGE_DIR "${NAV1CM_CORE}")
ament_python_install_package(${PROJECT_NAME})

add_library(nav1cm_terrain SHARED "${NAV1CM_CORE}/terrain_bridge.cpp")
target_compile_features(nav1cm_terrain PRIVATE cxx_std_20)
target_link_libraries(nav1cm_terrain PRIVATE
  lunar_incremental_navigation_core::lunar_incremental_navigation_core)
add_library(nav1cm_geometry SHARED "${NAV1CM_CORE}/native_geometry.cpp")
target_compile_features(nav1cm_geometry PRIVATE cxx_std_20)
add_library(nav1cm_distance_field SHARED "${NAV1CM_CORE}/distance_field_native.cpp")
target_compile_features(nav1cm_distance_field PRIVATE cxx_std_20)
install(TARGETS nav1cm_terrain nav1cm_geometry nav1cm_distance_field
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(PROGRAMS scripts/planner_node scripts/navigation_task_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../config/wheel.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(nav1cm_shutdown test/test_shutdown.py TIMEOUT 20)
  ament_add_pytest_test(nav1cm_runtime test/test_runtime.py TIMEOUT 30)
  ament_add_pytest_test(nav1cm_task_runtime test/test_navigation_task_runtime.py TIMEOUT 30)
  ament_add_pytest_test(nav1cm_task_action test/test_navigation_task_action.py TIMEOUT 45)
  ament_add_pytest_test(nav1cm_configuration test/test_configuration.py TIMEOUT 30)
endif()
ament_package()
