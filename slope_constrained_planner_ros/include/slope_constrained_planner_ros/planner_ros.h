/**
 * @file planner_ros.h
 * @brief An inheritance of the ``Planner`` class to be used in the ROS environment
 * @details
 * 
 *  **Subscribes to:**
 *    /elevation_mapping/elevation_map_raw
 * 
 *  **Publishes to:**
 *    /slope_constrained_planner/path
 * 
 *  **Uses Action:**
 *    /slope_constrained_planner/plan_to_goal
 */

#pragma once

#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <slope_constrained_planner/planner.h>
#include <slope_constrained_planner_msgs/PlanToGoalAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <slope_constrained_planner_ros/converter.h>
#include <slope_constrained_planner/params.h>


namespace slope_constrained_planner {


class PlannerRos : protected Planner {

 protected:

  // ROS members.
  using PlanningActionServer = actionlib::SimpleActionServer<slope_constrained_planner_msgs::PlanToGoalAction>;
  using Feedback = slope_constrained_planner_msgs::PlanToGoalFeedback;
  using FeedbackStatus = Feedback::_status_type;

  /// Node handler attribute
  ros::NodeHandle nh_;

  /// Map subscriber attribute
  ros::Subscriber map_sub_;

  /// Planner action server attribute
  std::unique_ptr<PlanningActionServer> plan_act_srv_;

  /// Planning time duration
  double planning_time_start_;
  double planning_time_end_;

  /// Path publisher attribute
  ros::Publisher path_pub_;

  /// Phi publisher attribute
  ros::Publisher phi_pub_;

  /// Theta publisher attribute
  ros::Publisher theta_pub_;

  /// Map publisher attribute
  ros::Publisher map_pub_;

  /// tf buffer attribute
  tf2_ros::Buffer tf_buffer_;

  /// tf listener attribute
  tf2_ros::TransformListener tf_listener_{tf_buffer_};


  /// Goal position attribute
  geometry_msgs::PoseStamped pose_goal_;

  /// Goal position process mutex attribute
  mutable std::mutex pose_goal_mutex_;

  ///  Set the planner thread to whether perform or not
  std::atomic<bool> planning_continuously_{false};

  /// Set the planning thread built status 
  std::atomic<bool> thread_built_{false};

  /// Sets the publish path thread to whether perform or not
  std::atomic<bool> publish_path_{false};

  /// Planner thread attribute
  std::thread continuous_planning_thread_;

  /// Publish path thread attribute
  std::thread publish_path_thread_;

  /// Publish phi thread attribute
  std::thread publish_phi_thread_;

  /// Planner thread process mutex attribute
  std::mutex planning_thread_mutex_;

  /// Found solution by the planner thread attribute
  nav_msgs::Path plan_ros_;

  /// Grid map info attribute
  grid_map_msgs::GridMapInfo planning_map_info_;

  /// Grid map data queue attribute
  struct {
    std::unique_ptr<grid_map::GridMap> map;
    grid_map_msgs::GridMapInfo info;
  } map_queue_;

  /// Grid map process mutex attribute
  mutable std::mutex map_queue_mutex_;

  /// An object from converter class attribute
  Converter converter_;


  /**
   * @brief Stops the planning thread by putting false value in the corresponding class attribute
  */
  void stopPlanningContinuously();


  /**
   * @brief The main thread in this class which tries to plan whithin the defined time from robot's
   * current position to the target point until a valid path is found
  */
  virtual void planContinuouslyThread();


  /**
   * @brief Publishes the path to corresponding topic continusly, once a valid path is found
   * by the planner thread
  */
  virtual void publishPathThread();

  /**
   * @brief Publishes phi to corresponding topic continusly.
  */
  void publishPhiThread();


  /**
   * @brief Tries to plan from current robot's pose to the target using
   * ``UpdateMapAndPlanFromCurrentRobotPose()`` function
  */
  virtual void planFromCurrentRobotPose();


  /**
   * @brief Gets robot current position from the tf tree
   * @param pose Robot position
  */
  bool getCurrentRobotPose(geometry_msgs::PoseStamped *pose) const;


  /**
   * @brief Publishes the planner server feedback
   * @param feedback Feedback to be published
  */
  void publishFeedback(FeedbackStatus feedback) const;


  /**
   * @brief Callback function for subscribing the grid map topic
   * @param map_msg Grid map data
  */
  void mapCallback(const grid_map_msgs::GridMapConstPtr& map_msg);


  /**
   * @brief Callback function for canceling the goal in the planner server
  */
  void cancelGoalCallback();


  /**
   * @brief Callback function for when a new pose is available in the planner server
  */
  void goalCallback();


  /**
   * @brief Helper function to be used in ``goalCallback()`` function. Gets the goal and trigers the
   * planner thread to find the path
   * @param goal Goal message which is defiened in the custom action file
  */
  void goalPoseCallback(const slope_constrained_planner_msgs::PlanToGoalGoalConstPtr& goal);


  /**
   * @brief Publishes the found path to corresponding topic
   * @param path The path which is found by the planner
  */
  virtual void publishPath(nav_msgs::Path path);


  /**
   * @brief Publishes the value of phi in degrees
   * @param phi The value of phi
  */
  virtual void publishPhi(std_msgs::Float64 phi);


    /**
   * @brief Publishes the value of theta in degrees
   * @param theta The value of theta
  */
  virtual void publishTheta(std_msgs::Float64 theta);


  /***
   * @brief Updates the grid map data
  */
  void updateMap();


  /**
   * @brief Publiesh the map data to corresponding topic
  */
  void publishMap() const;


  /**
   * @brief Uses ``updateMap()`` function and then ``plan(start, goal)`` function from the base class. Once the path
   * is found, calls ``publishMap()`` function. This function is used in the ``planContinueslyThread()`` function.
   * @param goal The goal state to be set in the planner
  */
  PlannerStatus updateMapAndPlanFromCurrentRobotPose(const ob::ScopedState<>& goal);

public:

  /**
   * @brief Destructor
  */
  ~PlannerRos();


  /**
   * @brief Constructor
   * @param nh NodeHandler object
  */
  explicit PlannerRos(const ros::NodeHandle& nh);

};



}
