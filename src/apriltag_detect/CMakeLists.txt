cmake_minimum_required(VERSION 3.5)
project(apriltag_detect)

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
find_package(VISP REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(node_plugins "")
set(APRILTAG_DETECT_COMPONENT_SRCS
  src/apriltag_detect_component.cpp)
set(APRILTAG_DETECT_COMPONENT_DEPS
  rclcpp
  sensor_msgs)


add_library(apriltag_detect_component SHARED ${APRILTAG_DETECT_COMPONENT_SRCS})
target_compile_definitions(apriltag_detect_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(apriltag_detect_component PUBLIC ${VISP_INCLUDE_DIRS})
target_link_libraries(apriltag_detect_component ${VISP_LIBRARIES})
ament_target_dependencies(apriltag_detect_component ${APRILTAG_DETECT_COMPONENT_DEPS})
rclcpp_components_register_nodes(apriltag_detect_component "visual_composition::ApriltagDetect")
set(node_plugins "${node_plugins}visual_composition::ApriltagDetect;$<TARGET_FILE:apriltag_detect_component>\n")

install(TARGETS
  apriltag_detect_component
)

install(
  DIRECTORY
  launch
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
