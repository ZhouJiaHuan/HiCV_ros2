cmake_minimum_required(VERSION 3.5)
project(face_detect)

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


set(node_plugins "")
set(FACE_DETECT_COMPONENT_SRCS
  src/ultra_face.cpp
  src/face_detect_component.cpp)
set(FACE_DETECT_COMPONENT_DEPS
  rclcpp
  sensor_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include/
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/
)

link_directories(/usr/local/lib)

add_library(face_detect_component SHARED ${FACE_DETECT_COMPONENT_SRCS})
target_compile_definitions(face_detect_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(face_detect_component ${FACE_DETECT_COMPONENT_DEPS})
target_link_libraries(face_detect_component MNN ${OpenCV_LIBS})
rclcpp_components_register_nodes(face_detect_component "visual_composition::FaceDetect")
set(node_plugins "${node_plugins}visual_composition::FaceDetect;$<TARGET_FILE:face_detect_component>\n")

install(TARGETS
  face_detect_component)

# Install various directories
install(
  DIRECTORY
  launch
  model
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
