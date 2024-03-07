/**
 * @file planner.h
 * @brief Implements the planner based on different path planning algorithms, which are provided in OMPL
 */

#pragma once

#include <mutex>

#include <grid_map_core/GridMap.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "slope_constrained_planner/map.h"
#include "slope_constrained_planner/params.h"
#include "slope_constrained_planner/planner_status.h"
#include "slope_constrained_planner/sampler.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace slope_constrained_planner {

/**
 * @brief Implements the planner based on different path planning algorithms, which are provided in OMPL
 * @details Desciption: The main class in the software architecture. Creates an object from the chosen planner,
 * sets the custom sampler and motion validator, and tries to find a path between the starting
 * and goal points in the provided grid map.
 * 
*/
class Planner {

protected:

  /// Simple setup attribute
  std::shared_ptr<og::SimpleSetup> ss_; 

  /// Contains some parameters such as planner name, verbose and map layers
  ParamsConstPtr params_;

  /// Status of planner
  bool solved_{false}; 

  /// Defines the states in se3 
  std::shared_ptr<ob::SE3StateSpace> space_;

  /// Defines the states in R(3)
  ob::StateSpacePtr space_real_vec_;

  /// An scoped state in R(3)
  ob::ScopedState<ob::RealVectorStateSpace> state_pos_;

  /// Grid map data
  std::shared_ptr<Map> map_;

  /// Grid map process mutex
  mutable std::mutex map_mutex_;

  /// Sets the custom sampler class
  SamplerAllocator sampler_allocator_;


public:

  /**
   * @brief Constructor for ``Planner`` class. Defines the state space and problem,
   * sets the custom sampler and motion validator, and the planner algorithm
   * 
   * @param params contains required info about the planner name
   */
  Planner(const ParamsConstPtr& params = std::make_shared<const Params>());


  /**
   * @brief Sets the grid map's different layers
   * 
   * @param map grid map of the environment
   */
  void setMap(std::unique_ptr<grid_map::GridMap>&& map);


  /**
   * @brief Plans a path between start and goal states
   * 
   * @param start Robots current state
   * @param goal  Chosen target state
   * 
   * @return PlannerStatus
   */
  PlannerStatus plan(const ob::ScopedState<>& start,
                     const ob::ScopedState<>& goal);


  /**
   * @brief Plans a path between start and goal states
   * 
   * @param simplify Specifies whether the path would be simplified or not (default = false)
   * 
   * @return PathGeometric Found solution path computed by the geometric planner
   */
  og::PathGeometric getSolutionPath(const bool& simplify = false) const;
};



}
