cmake_minimum_required(VERSION 3.8)
project(semantic_inference_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(semantic_inference REQUIRED)
find_package(semantic_inference_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
)

# ROS execs
add_executable(segmentator_node
              src/segmentator_node.cpp
              )
ament_target_dependencies(segmentator_node rclcpp sensor_msgs image_transport cv_bridge semantic_inference_msgs semantic_inference)
# target_link_libraries(
#   segmentator_node
#   PUBLIC rclcpp::rclcpp semantic_inference::semantic_inference
#   PRIVATE cv_bridge::cv_bridge
# )

# Binaries
install( TARGETS segmentator_node
          EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(rclcpp semantic_inference)
ament_package()
