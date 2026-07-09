cmake_minimum_required(VERSION 3.8)
project(arm_mission_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(grasp_executor_cpp
  src/grasp_executor_cpp.cpp
)

ament_target_dependencies(grasp_executor_cpp
  rclcpp
  std_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS grasp_executor_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/mono_trash_detector.py
    scripts/grasp_executor.py
    scripts/grasp_hw_bridge.py
    scripts/base_pick_assistant.py
    scripts/mock_camera_target.py
    scripts/debug_grasp_garbage.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
