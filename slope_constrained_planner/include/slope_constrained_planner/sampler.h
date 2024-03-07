/**
 * @file sampler.h
 * @brief Defines a custom sampler which only samples an state with the heading angle below the threshold
 */

#pragma once

#include <mutex>

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "slope_constrained_planner/map.h"


namespace ob = ompl::base;


namespace slope_constrained_planner {

/**
 * @brief Defines a custom sampler which only samples an state with the heading angle below the threshold
 * @details Description: A custom sampler which randomly samples a state position within map bounderies. Regarding the state
 * orientation, the yaw angle would be chosen randomly within (-pi, pi), roll and pitch angles are found
 * by getting the normal vector of the state cell in the map. In order to get the desired orientation,
 * state frame rotates along its z-axis until the heading angle gets below the threshold.
*/
class Sampler : public ob::StateSampler {

  /// Parameters containing the data for map layers
  ParamsConstPtr params_;

  /// Defines the states in R(3)
  ob::StateSpacePtr space_real_vec_;

  /// Defines the states in SO3
  ob::StateSpacePtr space_rot_;

  /// Sets R(3) state space bounderies
  std::shared_ptr<ob::RealVectorBounds> bounds_se3_;

  /// An scoped state in R(3)
  ob::ScopedState<ob::RealVectorStateSpace> state_pos_;

  /// Represents the position of previously found valid state
  grid_map::Position prev_pos_;

  /// An attribute to specify if the found sample is near robot
  bool sample_is_near_robot_;

  /// An scopes state in SE3
  ob::ScopedState<ob::SE3StateSpace> state_rot_;

  /// State sampler in R(2)
  ob::RealVectorStateSampler base_real_vec_;

  /// Grid map data
  std::shared_ptr<Map> map_;

  /// Grid map process mutex
  mutable std::mutex map_mutex_;

  /// specifies whether the previously found state was valid
  bool prev_state_is_valid_;

  /// Represents the yaw angle for previous valid state
  double prev_yaw_;

  /// Represents the heading angle for previous valid state
  double prev_phi_;

  /**
   * @brief Helper function to sample the state position within map bounderies and finit normal vector
   * 
   * @return pos state position info
   */
  grid_map::Position samplePositionInMap();

public:

  /**
   * @brief Constructor for ``Sampler`` class. Defines the space bounderies using the grid map
   * info.
   * 
   * @param space state space in which the sampler would find the states
   * @param map grid map of the environment
   * @param params contains required info about the map layers
   */
  Sampler(const ob::StateSpace* si,
                    const std::shared_ptr<Map>& map,
                    const ParamsConstPtr& params);

  
  /**
   * @brief Overriden function from the base class ``StateSampler``, uses ``samplePositionInMap()`` function
   * to find state position. Uses ``getNormal(ind)`` function to find roll and pitch angles for the state 
   * orientation. Keeps rotating the state frame along its z-axis until the heading angle gets below threshold.
   * If previously found state was valid, use its reverse orientation for the next state.
   * 
   * @param state The sampled state data
   */
  virtual void sampleUniform(ob::State* state) override;


  /**
   * @brief Not implemented in this software architecture
   */
  virtual void sampleUniformNear(ob::State* /*state*/,
                                 const ob::State* /*near*/,
                                 double /*distance*/) override;


  /**
   * @brief Not implemented in this software architecture
   */
  virtual void sampleGaussian(ob::State* /*state*/,
                              const ob::State* /*mean*/,
                              double /*std_dev*/) override;


  bool checkSampleDistance(const ob::ScopedState<>& sample);
};

/**
 * @class SamplerAllocator
 * @brief Allocates the custom sampler class in the planner
*/
class SamplerAllocator {

  /// Parameters containing the data for map layers
  ParamsConstPtr params_;

  /// Grid map data
  std::shared_ptr<Map> map_;

  public:

    SamplerAllocator(const ParamsConstPtr& params) :
      params_(params) {}


    /**
     * @brief Sets the grid map data in the corresponding class attribute
     * 
     * @param map grid map of the environment
     */
    void setMap(const std::shared_ptr<Map>& map);


    /**
     * @brief Gets the custom sampler class to allocate it in the planner 
     * 
     * @param space state space in which the sampler would find the states
     * @return Sampler a shared pointer to the custom sampler class
     */
    std::shared_ptr<Sampler> getSampler(const ob::StateSpace* space);
};


}
