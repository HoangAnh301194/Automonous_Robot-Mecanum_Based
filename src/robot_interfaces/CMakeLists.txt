cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/WaterStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/CustomerStatus.msg"
  "msg/TrackedPerson.msg"
  "msg/TrackedPersonArray.msg"
  "srv/SaveLocation.srv"
  "srv/GetLocation.srv"
  "srv/DeleteLocation.srv"
  "srv/GetLocationList.srv"
  "srv/SetLanguage.srv"
  "srv/FinishSetup.srv"
  "srv/SaveRoute.srv"
  "srv/GetRoute.srv"
  "action/Patrol.action"
  "action/DirectGo.action"
  DEPENDENCIES action_msgs std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
