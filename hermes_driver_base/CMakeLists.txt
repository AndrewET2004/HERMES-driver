cmake_minimum_required(VERSION 3.10)
project(hermes_driver_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── GPIO Library ─────────────────────────────────────────────────────
# pigpio is a C library; find it as a system library.
# On Raspberry Pi: sudo apt install libpigpio-dev
find_library(PIGPIO_LIB pigpio REQUIRED)

# ── Build ────────────────────────────────────────────────────────────
add_executable(diff_drive_controller_node
  src/diff_drive_controller.cpp
  src/tb6612fng.cpp
)

target_include_directories(diff_drive_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(diff_drive_controller_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(diff_drive_controller_node ${PIGPIO_LIB})

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS diff_drive_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()