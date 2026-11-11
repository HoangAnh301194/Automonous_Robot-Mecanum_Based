cmake_minimum_required(VERSION 3.8)
project(nav2_custom_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(nav2_custom_backup_evaluators SHARED
  src/rear_safety_evaluator.cpp
  src/custom_backup_evaluator.cpp
)
target_include_directories(nav2_custom_backup_evaluators PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_custom_backup_evaluators
  rclcpp
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  tf2
)

add_library(nav2_operator_alert_plugin SHARED src/operator_alert.cpp)
target_include_directories(nav2_operator_alert_plugin PRIVATE include)
ament_target_dependencies(nav2_operator_alert_plugin
  rclcpp nav2_core nav2_behavior_tree behaviortree_cpp_v3 diagnostic_msgs
)
target_compile_definitions(nav2_operator_alert_plugin PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_speak_plugin SHARED src/speak.cpp)
target_include_directories(nav2_speak_plugin PRIVATE include)
ament_target_dependencies(nav2_speak_plugin
  rclcpp nav2_core nav2_behavior_tree behaviortree_cpp_v3 std_msgs
)
target_compile_definitions(nav2_speak_plugin PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_check_rear_safety_plugin SHARED src/check_rear_safety.cpp)
target_include_directories(nav2_check_rear_safety_plugin PRIVATE include)
target_link_libraries(nav2_check_rear_safety_plugin nav2_custom_backup_evaluators)
ament_target_dependencies(nav2_check_rear_safety_plugin
  rclcpp nav2_core nav2_behavior_tree behaviortree_cpp_v3 sensor_msgs diagnostic_msgs tf2 tf2_ros
)
target_compile_definitions(nav2_check_rear_safety_plugin PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_recovery_progress_guard_plugin SHARED src/recovery_progress_guard.cpp)
target_include_directories(nav2_recovery_progress_guard_plugin PRIVATE include)
ament_target_dependencies(nav2_recovery_progress_guard_plugin
  rclcpp nav2_core nav2_behavior_tree behaviortree_cpp_v3 tf2_ros tf2_geometry_msgs
)
target_compile_definitions(nav2_recovery_progress_guard_plugin PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_custom_backup_behavior SHARED src/custom_backup.cpp)
target_include_directories(nav2_custom_backup_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(nav2_custom_backup_behavior nav2_custom_backup_evaluators)
ament_target_dependencies(nav2_custom_backup_behavior
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  nav2_util
  pluginlib
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_core custom_backup_plugins.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  nav2_custom_backup_evaluators
  nav2_operator_alert_plugin
  nav2_speak_plugin
  nav2_check_rear_safety_plugin
  nav2_recovery_progress_guard_plugin
  nav2_custom_backup_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_custom_backup test/test_custom_backup.cpp)
  if(TARGET test_custom_backup)
    target_include_directories(test_custom_backup PRIVATE include)
    target_link_libraries(test_custom_backup
      nav2_custom_backup_evaluators
      nav2_custom_backup_behavior
    )
    ament_target_dependencies(test_custom_backup
      rclcpp
      rclcpp_action
      rclcpp_lifecycle
      nav2_core
      nav2_behaviors
      nav2_behavior_tree
      behaviortree_cpp_v3
      nav2_msgs
      nav_msgs
      nav2_util
      pluginlib
      sensor_msgs
      diagnostic_msgs
      geometry_msgs
      tf2
      tf2_ros
    )
  endif()
endif()

ament_package()
