/**
 * @file utils.h
 * @brief Some helper functions to be used in different components of the software architecture
 */

# pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <ompl/base/spaces/SE3StateSpace.h>


namespace ob = ompl::base;


namespace slope_constrained_planner {

extern std::vector<double> global_start_;

using Scalar = float;
using Pose3 = Eigen::Transform<Scalar,3,Eigen::Affine>;


/**
 * @brief Gets position of state from its SE3 representation
 * @param state State info
 * @return Position of state
*/
inline Pose3 Pose3FromSE3(const ob::State* state) {
  const auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
  const auto& rotation = state_se3->rotation();
  Pose3 pose;
  pose.translation().x() = state_se3->getX();
  pose.translation().y() = state_se3->getY();
  pose.translation().z() = state_se3->getZ();
  pose.matrix().topLeftCorner(3,3) =
      Eigen::Quaternion<Scalar>(rotation.w,
                                rotation.x,
                                rotation.y,
                                rotation.z).toRotationMatrix();
  return pose;
}


/**
 * @brief Gets the position of state from its x-y-z representation
 * @param x State's x value
 * @param y State's y value
 * @param z State's z value
 * @return Position of the state
*/
inline Pose3 Pose3FromXYZ(Scalar x, Scalar y, Scalar z) {
  Pose3 pose = Pose3::Identity();
  pose.translation().x() = x;
  pose.translation().y() = y;
  pose.translation().z() = z;
  return pose;
}


/**
 * @brief Gets the lateral distance between two states
 * @param from Starting state
 * @param to Ending state
 * @return Lateral distance between from and to states
*/
inline double lateralDistance(const ob::State* from,
                              const ob::State* to) {
  const auto from_se3 = from->as<ob::SE3StateSpace::StateType>();
  const auto to_se3 = to->as<ob::SE3StateSpace::StateType>();

  const auto dx = to_se3->getX() - from_se3->getX();
  const auto dy = to_se3->getY() - from_se3->getY();

  return std::sqrt(dx*dx + dy*dy);
}


/**
 * @brief Gets roll angle from quaternion represntation
 * @param w Quaternion w value
 * @param x Quaternion x value
 * @param y Quaternion y value
 * @param z Quaternion z value
 * @return Roll angle
*/
template <typename T>
inline T getRollFromQuat(T w, T x, T y, T z) {
  return atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
}


/**
 * @brief Gets pitch angle from quaternion represntation
 * @param w Quaternion w value
 * @param x Quaternion x value
 * @param y Quaternion y value
 * @param z Quaternion z value
 * @return Pitch angle
*/
template <typename T>
inline T getPitchFromQuat(T w, T x, T y, T z) {
  return asin(2*(w*y - x*z));
}


/**
 * @brief Gets yaw angle from quaternion represntation
 * @param w Quaternion w value
 * @param x Quaternion x value
 * @param y Quaternion y value
 * @param z Quaternion z value
 * @return Yaw angle
*/
template <typename T>
inline T getYawFromQuat(T w, T x, T y, T z) {
  return atan2(2*(w*z + x*y), 1-2*(y*y + z*z));
}


/**
 * @brief Gets yaw angle from SO3 represntation
 * @param s State data
 * @return Yaw angle
*/
inline Scalar getYawFromSO3(const ob::SO3StateSpace::StateType& s) {
  return getYawFromQuat(s.w, s.x, s.y, s.z);
}


/**
 * @brief Gets roll angle from SO3 represntation
 * @param s State data
 * @return Roll angle
*/
inline Scalar getRollFromSO3(const ob::SO3StateSpace::StateType& s) {
  return getRollFromQuat(s.w, s.x, s.y, s.z);
}


/**
 * @brief Gets pitch angle from SO3 represntation
 * @param s State data
 * @return Pitch angle
*/
inline Scalar getPitchFromSO3(const ob::SO3StateSpace::StateType& s) {
  return getPitchFromQuat(s.w, s.x, s.y, s.z);
}


/**
 * @brief Gets SO3 represntation from yaw angle 
 * @param s State data
 * @param yaw Yaw angle
*/
inline void setSO3FromYaw(ob::SO3StateSpace::StateType& s, double yaw) {
  s.w = cos(yaw);
  s.x = 0;
  s.y = 0;
  s.z = sin(yaw);
}


/**
 * @brief Sets SO3 represntation using RPY representation
 * @param s State data
 * @param rpy Roll, pitch and yaw angles
*/
inline void setSO3FromRPY(ob::SO3StateSpace::StateType& s, double* rpy) {
  const auto r2 = rpy[0]*0.5;
  const auto p2 = rpy[1]*0.5;
  const auto y2 = rpy[2]*0.5;
  const auto cr = cos(r2);
  const auto cp = cos(p2);
  const auto cy = cos(y2);
  const auto sr = sin(r2);
  const auto sp = sin(p2);
  const auto sy = sin(y2);
  s.w = cy * cp * cr + sy * sp * sr;
  s.x = cy * cp * sr - sy * sp * cr;
  s.y = sy * cp * sr + cy * sp * cr;
  s.z = sy * cp * cr - cy * sp * sr;
}


/**
 * @brief Estimates the normal vectors in a grid map cell
 * @param map Grid map data
 * @param estimation_radius The dimension of estimation
 * @param input_layer Name of the input layer
 * @param output_layer_prefix Prefix of output layer
*/
void estimateNormals(grid_map::GridMap& map,
                     double estimation_radius,
                     const std::string& input_layer,
                     const std::string& output_layer_prefix = "normal");

}
