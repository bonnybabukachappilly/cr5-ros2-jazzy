cmake_minimum_required(VERSION 3.8)
project(cr5_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/CartesianPose.msg"
  "srv/EnableRobot.srv"
  "srv/DisableRobot.srv"
  "srv/EmptyRequest.srv"
  "srv/ClearError.srv"
  "srv/SetSpeed.srv"
  "srv/SetPayload.srv"
  "srv/SetMotionParams.srv"
  "srv/MoveJ.srv"
  "srv/RequestControl.srv"
  "srv/RobotMode.srv"
  DEPENDENCIES std_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
