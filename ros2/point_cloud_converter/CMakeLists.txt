cmake_minimum_required(VERSION 3.8)
project(point_cloud_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(point_cloud_converter_node src/converter.cpp)
# Converting the deprecated sensor_msgs/PointCloud is this node's purpose.
target_compile_options(point_cloud_converter_node PRIVATE -Wno-deprecated-declarations)
target_link_libraries(point_cloud_converter_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS point_cloud_converter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
