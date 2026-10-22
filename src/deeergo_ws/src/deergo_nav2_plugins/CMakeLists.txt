cmake_minimum_required(VERSION 3.8)
project(deergo_nav2_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(deergo_queue_layers SHARED src/queue_layers.cpp)
target_compile_features(deergo_queue_layers PUBLIC cxx_std_17)
target_include_directories(deergo_queue_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(deergo_queue_layers
  message_filters pluginlib rclcpp rclcpp_lifecycle rmw sensor_msgs tf2_ros)
target_link_libraries(deergo_queue_layers nav2_costmap_2d::layers)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS deergo_queue_layers EXPORT export_deergo_queue_layers
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_deergo_queue_layers HAS_LIBRARY_TARGET)
ament_export_dependencies(nav2_costmap_2d message_filters pluginlib
  rclcpp rclcpp_lifecycle rmw sensor_msgs tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_util REQUIRED)
  ament_add_gtest(test_queue_layers test/test_queue_layers.cpp)
  target_link_libraries(test_queue_layers deergo_queue_layers)
  ament_target_dependencies(test_queue_layers nav2_util)
endif()

ament_package()
