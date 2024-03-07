/**
 * @file utils.h
 * @brief Some helper functions to load and configure ROS params
 */

#pragma once

#include <string>

#include <slope_constrained_planner/params.h>
#include <ros/node_handle.h>


namespace slope_constrained_planner {


template <typename T>
/**
 * @brief Gets an specific ROS param name and sets its default value in a node handler
 * @param nh Node handler
 * @param name Param's name
 * @param default_val Param's default value
*/
inline T getParamWithDefaultWarning(const ros::NodeHandle& nh,
                             const std::string& name,
                             const T& default_val) {
  T param;

  if (!nh.param(name, param, default_val)) {
    ROS_WARN_STREAM("Could not find ROS param \"" << name <<
                    "\", set to default: " << default_val);
  }
  return param;
}


template <>
/**
 * @brief Gets an specific ROS param name and sets its default value in a node handler
 * @param nh Node handler
 * @param name Param's name
 * @param default_val Param's default value
*/
inline unsigned int getParamWithDefaultWarning<unsigned int>(const ros::NodeHandle& nh,
                                                      const std::string& name,
                                                      const unsigned int& default_val) {
  return getParamWithDefaultWarning(nh, name, static_cast<int>(default_val));
}


/**
 * @brief Gets the provided params in ``config`` directory and sets them in the node handler
 * using ``getParamWithDefaultWarning(nh, name, default_val)`` funciton
 * @param nh Node handler
*/
ParamsPtr loadRosParameters(const ros::NodeHandle& nh);


}
