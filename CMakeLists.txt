cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_driver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  swarm_uav_interfaces
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_planner_driver
  CATKIN_DEPENDS roscpp geometry_msgs swarm_uav_interfaces mavros_msgs
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
  src/polynomial_traj.cpp
  src/safety_predictor.cpp
  src/safety_logger.cpp
  src/emergency_brake.cpp
  src/yield_candidates.cpp
)
target_link_libraries(ego_planner_core ${catkin_LIBRARIES})

# 节点实现单独成库，供可执行文件与单测共同链接（状态机逻辑可注入时钟测试）
add_library(ego_planner_driver_node_lib
  src/ego_planner_driver_node.cpp
)
target_link_libraries(ego_planner_driver_node_lib ego_planner_core ${catkin_LIBRARIES})

add_executable(ego_planner_driver_node src/ego_planner_driver_node_main.cpp)
target_link_libraries(ego_planner_driver_node ego_planner_driver_node_lib ego_planner_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory ego_planner_driver_node_lib ego_planner_core ${catkin_LIBRARIES})
  catkin_add_gtest(test_ego_state_machine test/test_ego_state_machine.cpp)
  target_link_libraries(test_ego_state_machine ego_planner_driver_node_lib ego_planner_core ${catkin_LIBRARIES})
  catkin_add_gtest(test_safety_predictor test/test_safety_predictor.cpp)
  target_link_libraries(test_safety_predictor ego_planner_core ${catkin_LIBRARIES})
  catkin_add_gtest(test_yield_candidates test/test_yield_candidates.cpp)
  target_link_libraries(test_yield_candidates ego_planner_core ${catkin_LIBRARIES})
  catkin_add_gtest(test_emergency_brake test/test_emergency_brake.cpp)
  target_link_libraries(test_emergency_brake ego_planner_core ${catkin_LIBRARIES})
  catkin_add_gtest(test_closed_loop_dynamics test/test_closed_loop_dynamics.cpp)
  target_link_libraries(test_closed_loop_dynamics ego_planner_core ${catkin_LIBRARIES})
  # 短末段内存回归（implementation_plan_26082722 §13.1）：父进程 fork 受限 worker
  # （RLIMIT_AS / RLIMIT_CPU / wall timeout），统计分类矩阵与 RSS 检查点。
  catkin_add_gtest(test_short_final_segment test/test_short_final_segment.cpp)
  target_link_libraries(test_short_final_segment ego_planner_driver_node_lib ego_planner_core ${catkin_LIBRARIES})
endif()
