cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(object_detection_dbscan src/dbscan.cpp)
target_compile_features(object_detection_dbscan PUBLIC cxx_std_17)
target_include_directories(object_detection_dbscan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(object_detection_node src/object_detection_node.cpp)
target_compile_features(object_detection_node PUBLIC cxx_std_17)
target_link_libraries(object_detection_node object_detection_dbscan)
ament_target_dependencies(object_detection_node
  geometry_msgs
  interfaces
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS object_detection_dbscan object_detection_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dbscan test/test_dbscan.cpp)
  target_link_libraries(test_dbscan object_detection_dbscan)
endif()

ament_export_include_directories(include)
ament_export_libraries(object_detection_dbscan)
ament_package()
