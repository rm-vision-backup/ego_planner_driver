cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_driver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  swarm_uav_interfaces
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_planner_driver
  CATKIN_DEPENDS roscpp geometry_msgs swarm_uav_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_library(ego_planner_core
  src/ego_planner_core.cpp
  src/uniform_bspline.cpp
)
target_link_libraries(ego_planner_core ${catkin_LIBRARIES})

add_executable(ego_planner_driver_node src/ego_planner_driver_node.cpp)
target_link_libraries(ego_planner_driver_node ego_planner_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory ego_planner_core ${catkin_LIBRARIES})
endif()