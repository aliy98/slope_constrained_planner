/**
 * @file map.h
 * @brief Grid map data class. Represents the grid map of the environment containing the defined layers.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <grid_map_core/GridMap.hpp>

#include "slope_constrained_planner/utils.h"
#include "slope_constrained_planner/params.h"
#include "slope_constrained_planner/chain.h"


namespace slope_constrained_planner {


/// definition of an object from grid map class
using GridMapPtr = std::unique_ptr<grid_map::GridMap>;


/**
 * @brief Grid map data class. Represents the grid map of the environment containing the defined layers.
*/
class Map {

  /// Grid map object attribute
  GridMapPtr map_{new grid_map::GridMap()};

  /// Params containing the grid map layers info
  ParamsConstPtr params_;

  /// Grid map process mutex
  mutable std::mutex mutex_;

  /// Map pre-processor attribute to be used in setting map
  ChainNewMap map_pre_processor_;

  /// Old map post-processor attribute to be used in setting map
  ChainOldMap old_map_post_processor_;


public:

  /**
   * @brief Constructs an empty map, without underlying grid map.
   * @param params Contains grid map layers info
   * */ 
  Map(const ParamsConstPtr& params);


  /**
   * @brief Sets the underlying grid map by taking ownership of it.
   * @param map Grid map to be set
  */
  void setMap(GridMapPtr&& map);


  /**
   * @brief Copies the underlying map and parameters of another map object.
   * @param map Grid map to be copied
  */
  void copy(const Map& map);


  /**
   * @brief Returns the layers available in the grid map
  */
  const std::vector< std::string > getLayers();


  /**
   * @brief Checks whether position is inside of map bounds.
   * @param pos Position of the state to br checked
   * @return Whether the state is inside the map or not
  */
  inline bool isInside(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->isInside(pos);
  }


  /**
   * @brief Gets the position of state with a particular index in the grid map
   * @param ind Index of the state in the grid map
   * @return pos Position of the state
  */
  grid_map::Position getPositionOfIndex(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::Position pos;
    map_->getPosition(ind, pos);
    return pos;
  }


  /**
   * @brief Gets the height value at a certain map index.
   * @param ind Index of the state in the grid map
   * @return Height of state
  */
  inline float getHeightAtIndex(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->at(params_->planner.elevation_layer, ind);
  }


  /**
   * @brief Gets the height value at a certain map position. Might throw exceptions if 
   * position is outside map boundaries.
   * @param pos Position of the state
   * @return Height of state position
  */
  inline float getHeightAtPosition(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->atPosition(params_->planner.elevation_layer, pos);
  }

  
  /**
   * @brief Return whether this cell was updated in the last map update. 
   * Might throw exceptions if position is outside map boundaries.
   * @param pos Position of the state
  */
  inline bool getUpdatedAtPosition(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->atPosition("updated", pos) > std::numeric_limits<grid_map::DataType>::epsilon();
  }


  /**
   * @brief Updates the grid map data from and starting to the ending position 
   * @param start Starting position
   * @param end Ending position
   * @return Whether the grid map data is updated
  */
  bool getUpdatedOnLine(const grid_map::Position& start, const grid_map::Position& end);


  /**
   * @brief  Gets the terrain normal at a certain map index.
   * @param ind Index of state in the grid map
   * @return Normal Vector at the index
   * */
  inline Eigen::Vector3d getNormal(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    estimateNormals(*map_, (params_->robot.torso.length + params_->robot.torso.width)*.75, params_->planner.elevation_layer);
    return Eigen::Vector3d(map_->at("normal_x", ind),
                           map_->at("normal_y", ind),
                           map_->at("normal_z", ind));
  }


  /** 
   * @brief Gets the map index of a certain position. Might throw exception if position is out of bounds.
   * @param pos Position of the state 
   * @return ind Index of position
   * */
  inline grid_map::Index getIndexOfPosition(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::Index ind;
    if (!map_->getIndex(pos, ind)) {
      throw std::out_of_range("Requested position is outside of map bounds.");
    }
    return ind;
  }


  /**
   * @brief Gets the standard deviation of a plane fit error at the given map index.
   * @param ind Index of the state at the grid map
   * @return Standard deviation of plane
  */
  inline float getPlaneFitStdDev(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->at("plane_fit_std_dev", ind);
  }


  /**
   * @brief Gets the required layer from the grid map
   * @param layer Required layer name
   * @return Layer data
  */
  inline const grid_map::Matrix& getLayer(const std::string& layer) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->get(layer);
  }


  /**
   * @brief Adds a layer to the grid map
   * @param layer_name Name of the layer to be added
   * @param data Layer data
  */
  inline void addLayer(const std::string& layer_name, const grid_map::Matrix& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_->add(layer_name, data);
  }


  /**
   * @brief Adds a layer to the grid map
   * @param layer_name Name of the layer to be added
   * @param val Layer value
  */
  inline void addLayer(const std::string& layer_name, const double val) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_->add(layer_name, val);
  }


  /**
   * @brief Gets the length to cell resolution ratio of the grid map
   * @param length Length to be checked
   * @return Length to cell 
   * 
  */
  inline Eigen::Index lengthToCellCount(const double& length) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return length / map_->getResolution();
  }


  /**
   * @brief Gets the map from the corresponding class attribute
   * @return Grid map data
  */
  inline const grid_map::GridMap& getMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return *map_;
  }


  /**
   * @brief Performs the pre-processing on the map again
  */
  void reApplyPreprocessing();
};


}
