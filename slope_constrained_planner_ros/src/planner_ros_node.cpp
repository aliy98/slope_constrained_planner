/**
 * @file planner_ros_node.cpp
 * @brief Executable main file for slope constrained planner in ROS environment
 */
#include "slope_constrained_planner_ros/planner_ros.h"

using namespace slope_constrained_planner;

/**
 * @brief Main function for ``slope_constrained_planner`` node
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "slope_constrained_planner");

  ros::NodeHandle nh("~");
  PlannerRos planner(nh);

  ros::AsyncSpinner spinner(nh.param<int>("planner/n_threads",1));

  spinner.start();
  ros::waitForShutdown();
}
