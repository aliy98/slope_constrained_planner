/**
 * @file map.cpp
* @brief Contains the definition of public member functions of `Map` class
 */

#include "slope_constrained_planner/map.h"
#include "slope_constrained_planner/utils.h"

#include <random>

#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/SubmapGeometry.hpp>


using namespace slope_constrained_planner;


void Map::setMap(GridMapPtr&& map_new) {
  map_new->setBasicLayers({params_->planner.elevation_layer});
  map_pre_processor_.process(map_new);
  if (map_) {
    // Not locking mutex here should be fine because we only read from map_.
    old_map_post_processor_.process(map_new, map_);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const GridMapPtr map_old = std::move(map_);
  map_ = std::move(map_new);
}


void Map::copy(const Map& map) {
  std::lock_guard<std::mutex> lock(mutex_);
  *map_ = *map.map_;
  params_ = map.params_;
}


Map::Map(const ParamsConstPtr& params)
    : params_(params) {

}


bool Map::getUpdatedOnLine(const grid_map::Position& start, const grid_map::Position& end) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto updated = map_->get("updated");
  grid_map::LineIterator iter(*map_, start, end);
  for (; !iter.isPastEnd(); ++iter) {
    const auto ind = *iter;
    if (updated(ind.x(), ind.y()) > std::numeric_limits<grid_map::DataType>::epsilon()) return true;
  }
  return false;
}


void Map::reApplyPreprocessing() {
  map_pre_processor_.process(map_);
}


const std::vector< std::string > Map::getLayers() {
  auto layers = map_->getLayers();
  return layers;
}