/**
 * @file planner_ros.cpp
 * @brief Contains the definition of public member functions of ``PlannerRos`` class

 */

#include "slope_constrained_planner_ros/planner_ros.h"

#include <functional>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ros/spinner.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>

#include <slope_constrained_planner_ros/utils.h>


using namespace slope_constrained_planner;
using std::ofstream;


PlannerRos::PlannerRos(const ros::NodeHandle& nh)
    : Planner(loadRosParameters(nh)),
      nh_(nh),
      converter_(space_) {
  map_sub_ = nh_.subscribe("elevation_map", 1, &PlannerRos::mapCallback, this);
  plan_act_srv_ = std::make_unique<PlanningActionServer>(nh_, "plan_to_goal", false);
  plan_act_srv_->registerGoalCallback(std::bind(&PlannerRos::goalCallback, this));
  plan_act_srv_->registerPreemptCallback(std::bind(&PlannerRos::cancelGoalCallback, this));
  plan_act_srv_->start();
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
  phi_pub_ = nh_.advertise<std_msgs::Float64>("phi", 1, true);
  theta_pub_ = nh_.advertise<std_msgs::Float64>("theta", 1, true);
  publish_phi_thread_= std::thread(&PlannerRos::publishPhiThread, this);
  map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map", 1, true);
}


void PlannerRos::mapCallback(const grid_map_msgs::GridMapConstPtr& map_msg) {
  std::lock_guard<std::mutex> lock(map_queue_mutex_);

  if (!map_queue_.map) map_queue_.map.reset(new grid_map::GridMap());
  grid_map::GridMapRosConverter::fromMessage(*map_msg, *map_queue_.map);
  map_queue_.map->convertToDefaultStartIndex();

  map_queue_.info = map_msg->info;
}


void PlannerRos::stopPlanningContinuously() {
  planning_continuously_ = false;
  // std::lock_guard<std::mutex> lock(planning_thread_mutex_);
  // if (continuous_planning_thread_.joinable()) {
  //   continuous_planning_thread_.join();
  // }
}


void PlannerRos::planContinuouslyThread() {
  ros::Time last_plan_start;
  ros::Duration replan_time(1/params_->planner.replan_freq);

  while (ros::ok()) {
    while (planning_continuously_) {
      last_plan_start = ros::Time::now();

      ROS_INFO_STREAM("Planning Started!");
      planFromCurrentRobotPose();
      ROS_INFO_STREAM("Planning done!");

      const auto cur_time = ros::Time::now();
      const auto sleep_time = replan_time - (cur_time - last_plan_start);
      if (sleep_time.toSec() > 0.0) {
        // Sleep until we want to replan.
        sleep_time.sleep();
      }
    }
  }
}


void PlannerRos::publishPathThread() {
  ros::Rate r(1.0); // 10 hz

  while (publish_path_) {
    publishPath(plan_ros_);
    r.sleep();
  }
}

void PlannerRos::publishPhiThread() {
  geometry_msgs::PoseStamped pose_robot;
  Eigen::Vector3d i_s(1, 0, 0);
  Eigen::Vector3d j_s(0, 1, 0);
  std_msgs::Float64 phi;
  std_msgs::Float64 theta;
  ros::Rate r(1.0); // 10 hz

  double path_flwr_init_time = 0;
  double now_time = 0;
   ofstream outdata_phi;
  ofstream outdata_theta;
  std::string packagePath = ros::package::getPath("slope_constrained_planner_ros");
  packagePath = packagePath + "/data/";
  outdata_phi.open(packagePath + "phi.txt");
  outdata_theta.open(packagePath + "theta.txt");

  while(ros::ok()) {
    if (!getCurrentRobotPose(&pose_robot))
      ROS_WARN_STREAM("Could not get robot pose.");
    double q0 = pose_robot.pose.orientation.w;
    double q1 = pose_robot.pose.orientation.x;
    double q2 = pose_robot.pose.orientation.y;
    double q3 = pose_robot.pose.orientation.z;
    Eigen::Quaterniond R_ws(q0, q1, q2, q3);
    // Get phi and theta value
    auto i_w = R_ws* i_s;
    auto j_w = R_ws* j_s;
    phi.data = atan(i_w[2] / sqrt(i_w[0]*i_w[0] + i_w[1]*i_w[1]));
    phi.data = phi.data * 180 / M_PI;
    theta.data = atan(j_w[2] / sqrt(j_w[0]*j_w[0] + j_w[1]*j_w[1]));
    theta.data = theta.data * 180 / M_PI;
    publishPhi(phi);
    publishTheta(theta);
    if (ros::service::exists("/path_follower/dummy_service", false)) {
      ros::param::get("/path_follower/init_time", path_flwr_init_time);
      now_time = ros::Time::now().toSec() - path_flwr_init_time;
      outdata_phi << now_time << ", " << phi.data << std::endl;
      outdata_theta << now_time << ", " << theta.data << std::endl;
    }
    r.sleep();
  }
  outdata_phi.close();
  outdata_theta.close();
}


bool PlannerRos::getCurrentRobotPose(geometry_msgs::PoseStamped* pose) const {
  // Get robot pose.
  geometry_msgs::TransformStamped pose_tf;
  try {
    pose_tf = tf_buffer_.lookupTransform(map_queue_.info.header.frame_id,
                                    params_->robot.base_frame,
                                    ros::Time::now(),
                                    ros::Duration(10.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s",ex.what());
    ROS_ERROR("Could not get robot pose from TF. Cannot plan!");
    publishFeedback(Feedback::NO_ROBOT_TF);
    return false;
  }

  tf2::Stamped<tf2::Transform> tf_tf2;
  tf2::fromMsg(pose_tf, tf_tf2);

  // Get feet center pose from base.
  tf2::Transform offset;
  offset.setIdentity();
  offset.setOrigin(tf2::Vector3(0, 0, params_->robot.feet.offset.z));

  tf_tf2 *= offset;

  tf2::toMsg(tf_tf2, *pose);
  return true;
}


void PlannerRos::planFromCurrentRobotPose() {
  // Convert goal pose to map frame (map might drift w.r.t goal).
  geometry_msgs::PoseStamped pose_goal_transformed;
  try {
    std::lock_guard<std::mutex> lock(pose_goal_mutex_);
    // Set goal stamp to map time because goal might have very old stamp.
    pose_goal_.header.stamp = map_queue_.info.header.stamp;
    tf_buffer_.transform(pose_goal_,
                         pose_goal_transformed,
                         map_queue_.info.header.frame_id,
                         ros::Duration(0.1));
    ROS_INFO_STREAM("Goal pose converted to map frame.");
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s",ex.what());
    ROS_ERROR("Could not transform goal pose to map frame. Not planning.");
    publishFeedback(Feedback::NO_GOAL_TF);
    return;
  }

  ROS_INFO_STREAM("Converting goal pose info to OMPL base.");
  ob::ScopedState<> goal = converter_.poseRosToOmpl(pose_goal_transformed);

  publishFeedback(Feedback::PLANNING);
  const auto result = updateMapAndPlanFromCurrentRobotPose(goal);

  switch (result) {
    case PlannerStatus::INVALID_GOAL: publishFeedback(Feedback::INVALID_GOAL); ROS_ERROR_STREAM("Invalid goal!"); break;
    case PlannerStatus::INVALID_START: publishFeedback(Feedback::INVALID_START); ROS_ERROR_STREAM("Invalid start!"); break;
    case PlannerStatus::NO_MAP: publishFeedback(Feedback::NO_MAP); ROS_ERROR_STREAM("No map!"); break;
    case PlannerStatus::NOT_SOLVED: publishFeedback(Feedback::NO_SOLUTION); ROS_ERROR_STREAM("No solution!"); break;
    case PlannerStatus::SOLVED: publishFeedback(Feedback::FOUND_SOLUTION); ROS_INFO_STREAM("Solution found!"); break;
    case PlannerStatus::UNKNOWN: ROS_ERROR_STREAM("Unknown planner feedback. Something is wrong!");
  }

  // Publish path if successful and we did not reach the goal.
  if (result == PlannerStatus::SOLVED && planning_continuously_) {
    planning_time_end_ = ros::Time::now().toSec();
    plan_ros_ = converter_.pathOmplToRos(getSolutionPath(params_->planner.simplify_solution));
    plan_ros_.header.frame_id = planning_map_info_.header.frame_id;
    plan_ros_.header.stamp = ros::Time::now();

    if (!publish_path_){
      std::lock_guard<std::mutex> lock(planning_thread_mutex_);
      publish_path_ = true;
      publish_path_thread_ = std::thread(&PlannerRos::publishPathThread, this);
    }
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    og::PathGeometric path = ss_->getSolutionPath();
    std::cout << "Found solution:" << std::endl;
    ofstream outdata;
    std::string packagePath = ros::package::getPath("slope_constrained_planner_ros");
    packagePath = packagePath + "/data/";
    outdata.open(packagePath + "path.txt");
    outdata << "Planning time (sec): " << planning_time_end_ - planning_time_start_ <<std::endl;
    outdata << "Path data ([x, y, yaw]): " <<std::endl;
    for (int i=0; i<path.getStates().size(); i++) {
      auto yaw = getYawFromSO3(path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation());
      std::cout << "[ " << path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX() << ", ";
      outdata << "[ " << path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX() << ", ";
      std::cout << path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY() << ", ";
      outdata << path.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY() << ", ";
      std::cout << yaw << "]" << std::endl;
      outdata << yaw << "]" << std::endl;
    }
    outdata.close();
    stopPlanningContinuously();
    plan_act_srv_->setSucceeded();
  } else {
    ROS_ERROR_STREAM("Planning continusly is not active!"); 
  }
}


void PlannerRos::publishFeedback(FeedbackStatus feedback) const {
  Feedback feeback_msg;
  feeback_msg.status = feedback;
  plan_act_srv_->publishFeedback(feeback_msg);
}


void PlannerRos::cancelGoalCallback() {
  ROS_INFO_STREAM("Stop continuous planning requested.");
  stopPlanningContinuously();
  plan_act_srv_->setPreempted();
}


void PlannerRos::goalCallback() {
  planning_time_start_= ros::Time::now().toSec();
  goalPoseCallback(plan_act_srv_->acceptNewGoal());
}


void PlannerRos::goalPoseCallback(const slope_constrained_planner_msgs::PlanToGoalGoalConstPtr& goal_msg) {
  std::lock_guard<std::mutex> lock(pose_goal_mutex_);

  pose_goal_ = goal_msg->goal;
  ROS_INFO_STREAM("Received goal pose:\n" << pose_goal_);
  publishFeedback(Feedback::GOAL_RECEIVED);
  if (!planning_continuously_)
    planning_continuously_ = true;
  if (!thread_built_){
    thread_built_ = true;
    std::lock_guard<std::mutex> lock(planning_thread_mutex_);
    continuous_planning_thread_ = std::thread(&PlannerRos::planContinuouslyThread, this);
  }
}


void PlannerRos::publishPath(nav_msgs::Path path) {
  // Publish regular ROS message.
  path_pub_.publish(path);
}


void PlannerRos::publishPhi(std_msgs::Float64 phi) {
  // Publish regular ROS message.
  phi_pub_.publish(phi);
}


void PlannerRos::publishTheta(std_msgs::Float64 theta) {
  // Publish regular ROS message.
  theta_pub_.publish(theta);
}


PlannerRos::~PlannerRos() {
  // Wait for planning thread to finish.
  if (planning_continuously_) {
    std::cout << "Stopping continuous planning before shutdown." << std::endl;
    stopPlanningContinuously();
    publishFeedback(Feedback::NODE_SHUTDOWN);
    plan_act_srv_->setAborted();
  }
}


void PlannerRos::updateMap() {
  ROS_INFO_STREAM("Updating map info.");
  std::lock_guard<std::mutex> lock(map_queue_mutex_);

  if (!map_queue_.map) {
    ROS_WARN_STREAM("No new map received since last planning call.");
  } else {
    planning_map_info_ = map_queue_.info;
    setMap(std::move(map_queue_.map));
    ROS_INFO_STREAM("Map set succefully.");
  }
}


void PlannerRos::publishMap() const {
  if (map_pub_.getNumSubscribers() > 0) {
    grid_map_msgs::GridMap out_msg;
    grid_map::GridMapRosConverter::toMessage(map_->getMap(), out_msg);
    out_msg.info = planning_map_info_;
    map_pub_.publish(out_msg);
  } else {
    ROS_WARN_STREAM("Map did not publish.");
  }
}


PlannerStatus PlannerRos::updateMapAndPlanFromCurrentRobotPose(const ob::ScopedState<>& goal) {
  updateMap();
  ss_->clear();
  ROS_INFO_STREAM("Planner cach cleared.");
  ss_->setup();
  ROS_INFO_STREAM("Planner created.");

  ROS_INFO_STREAM("Getting robot pose.");
  // Get robot pose.
  geometry_msgs::PoseStamped pose_robot;
  if (!getCurrentRobotPose(&pose_robot)) {
    ROS_WARN_STREAM("Could not get robot pose.");
    return PlannerStatus::UNKNOWN;
  }

  ROS_INFO_STREAM("Robot pose updated.");
  ob::ScopedState<> start = converter_.poseRosToOmpl(pose_robot);

  ROS_INFO_STREAM("Planning.");
  const auto result = plan(start, goal);

  publishMap();

  return result;
}
