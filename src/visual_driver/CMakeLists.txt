cmake_minimum_required(VERSION 3.5)
project(visual_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(rclcpp_components REQUIRED)


# sources and dependences of camera component
set(CAMERA_COMPONENT_SRCS
  src/camera_component.cpp)
set(CAMERA_COMPONENT_DEPS
  OpenCV
  rclcpp
  std_msgs
  sensor_msgs
  camera_calibration_parsers)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include)


set(node_plugins "")

# library of camera component
add_library(camera_component SHARED
  ${CAMERA_COMPONENT_SRCS})
target_compile_definitions(camera_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(camera_component PUBLIC
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera_component
  ${CAMERA_COMPONENT_DEPS})
rclcpp_components_register_nodes(camera_component "visual_composition::Camera")
set(node_plugins "${node_plugins}visual_composition::Camera;$<TARGET_FILE:camera_component>\n")


install(TARGETS
  camera_component
)

install(
  DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
