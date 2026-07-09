cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_station)

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
    scripts/rviz_click_router.py
    scripts/setup_ssh_keys.py
    scripts/station_gui.py
    scripts/start_station_linux.sh
    scripts/start_station_windows.bat
    scripts/build_station_exe.bat
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
