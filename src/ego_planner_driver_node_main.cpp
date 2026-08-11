#include <ros/ros.h>
#include "ego_planner_driver/ego_planner_driver_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_planner_driver_node");
  ego_planner_driver::EgoPlannerDriverNode node;
  ros::spin();
  return 0;
}