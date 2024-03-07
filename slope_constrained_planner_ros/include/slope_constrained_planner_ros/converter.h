/**
 * @file converter.h
 * @brief Defines some helper functions to convert data from OMPL to ROS and vice versa
 */

#include <slope_constrained_planner/planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace slope_constrained_planner {

/**
 * @brief Helper class for converting data from OMPL to ROS and vice versa
*/
class Converter {

  std::shared_ptr<ob::SE3StateSpace> space_;

  public:

    /**
     * @brief Constructor
     * @param space State space of the problem represented in SE3
    */
    Converter(std::shared_ptr<ob::SE3StateSpace> space);


    /**
     * @brief Converts the pos data from type ``geometry_msgs::PoseStamped`` to SE3 state data type
     * @param pose_ros Position to be converted 
    */
    ob::ScopedState<> poseRosToOmpl(const geometry_msgs::PoseStamped& pose_ros) const;


    /**
     * @brief Converts the found path by the planner to ``nav_msgs::Path`` data type
     * @param path Found solution by the planner in ``og::PathGeometric`` format
    */
    nav_msgs::Path pathOmplToRos(const og::PathGeometric& path) const;

};



}
