cmake_minimum_required(VERSION 3.10)
project(hermes_motor_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ── GPIO Library ─────────────────────────────────────────────────────
# libgpiod v1.x C library for Linux GPIO character devices.
# On Raspberry Pi OS (Bullseye) / Ubuntu 22.04: sudo apt install libgpiod-dev
# Uses the v1 API (tested with 1.6.x); incompatible with the v2 API.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED gpiod)

# Threads are required for software PWM
find_package(Threads REQUIRED)

# ── Hardware interface plugin (shared library) ────────────────────────
add_library(hermes_hardware SHARED
  src/hermes_hardware.cpp
  src/tb6612fng.cpp
)

target_include_directories(hermes_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)

ament_target_dependencies(hermes_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(hermes_hardware ${GPIOD_LIBRARIES} Threads::Threads)

# Register the pluginlib description file
pluginlib_export_plugin_description_file(hardware_interface hermes_motor_base_plugin.xml)

# ── Legacy standalone node (kept for backward compatibility) ──────────
add_executable(diff_drive_controller_node
  src/diff_drive_controller.cpp
  src/tb6612fng.cpp
)

target_include_directories(diff_drive_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)

ament_target_dependencies(diff_drive_controller_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(diff_drive_controller_node ${GPIOD_LIBRARIES} Threads::Threads)

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS hermes_hardware
  EXPORT export_hermes_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS diff_drive_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES hermes_motor_base_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()