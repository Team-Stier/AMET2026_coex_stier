cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(planner_core
  src/planner_core.cpp
  src/cost_functions/distance.cpp
  src/cost_functions/min_curvature.cpp
  src/cost_functions/select.cpp
)
target_include_directories(planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(planner_core PUBLIC cxx_std_17)

add_executable(path_planning_node src/path_planning_node.cpp)
target_link_libraries(path_planning_node planner_core)
ament_target_dependencies(path_planning_node
  ament_index_cpp
  geometry_msgs
  interfaces
  nav_msgs
  rclcpp
)

install(TARGETS planner_core path_planning_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../rddf/"
  DESTINATION share/${PROJECT_NAME}/rddf
  FILES_MATCHING PATTERN "rddf*.csv"
)
install(PROGRAMS launch.sh DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_link_libraries(test_planner_core planner_core)
endif()

ament_package()
