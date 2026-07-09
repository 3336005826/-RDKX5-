cmake_minimum_required(VERSION 3.8)
project(car_mission_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/mission_manager.py
    scripts/coverage_path_planner.py
    scripts/station_map_relay.py
    scripts/region_goal_sender.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
