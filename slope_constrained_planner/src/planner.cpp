/**
 * @file planner.cpp
 * @brief Contains the definition of public member functions of `Planner` class
 */

#include <slope_constrained_planner/planner.h>

#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>

#include <slope_constrained_planner/motion_validator.h>
#include "slope_constrained_planner/utils.h"

namespace slope_constrained_planner {
  std::vector<double> global_start_ = {0.0};
}
using namespace slope_constrained_planner;
using namespace std::placeholders;


Planner::Planner(const ParamsConstPtr& params) 
    : params_(params),
      space_real_vec_(new ob::RealVectorStateSpace(3)),
      state_pos_(space_real_vec_),
      map_(std::make_shared<Map>(params)),
      sampler_allocator_{params_} {

    // Define State Space and Problem
    space_ = std::make_shared<ob::SE3StateSpace>();
    ss_ = std::make_shared<og::SimpleSetup>(space_);

    if (params_->verbose) {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEBUG);
    } else {
      ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
    }
    
    // Set motion validator
    auto si = ss_->getSpaceInformation();
    si->setMotionValidator(std::make_shared<SlopeConstrainedMotionValidator>(si, params_)); 

    // Set planner algorithm
    if (params_->planner.name == "prm_star") {
      auto constrained_planner(std::make_shared<og::PRMstar>(si));
      ss_->setPlanner(constrained_planner);
    } else if (params_->planner.name == "lazy_prm_star"){
      auto constrained_planner(std::make_shared<og::LazyPRMstar>(si));
      ss_->setPlanner(constrained_planner);
    } else if (params_->planner.name == "rrt_star"){
      auto constrained_planner(std::make_shared<og::LazyPRMstar>(si));
      ss_->setPlanner(constrained_planner);
    } else if (params_->planner.name == "rrt_sharp"){
      auto constrained_planner(std::make_shared<og::RRTsharp>(si));
      ss_->setPlanner(constrained_planner);
    }

    // Set sampler.
    sampler_allocator_.setMap(map_);
    space_->setStateSamplerAllocator(std::bind(&SamplerAllocator::getSampler,
                                                sampler_allocator_,
                                                std::placeholders::_1));
}


void Planner::setMap(std::unique_ptr<grid_map::GridMap>&& map) {

  if (!map->exists(params_->planner.elevation_layer)) {
    if (params_->verbose) {
      std::cout << "Grid map does not have \""
                << params_->planner.elevation_layer
                << "\" layer." << std::endl;
    }
    return;
  }

  const auto& pos = map->getPosition();
  const auto& length = map->getLength();
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, pos.x() - length.x());
  bounds.setLow(1, pos.y() - length.y());
  bounds.setLow(2, map->get(params_->planner.elevation_layer).minCoeffOfFinites()
                     - params_->robot.feet.reach.z/2);
  bounds.setHigh(0, pos.x() + length.x());
  bounds.setHigh(1, pos.y() + length.y());
  bounds.setHigh(2, map->get(params_->planner.elevation_layer).maxCoeffOfFinites()
                      + params_->robot.feet.reach.z/2);

  std::lock_guard<std::mutex> lock(map_mutex_);

  space_->setBounds(bounds);
  map_->setMap(std::move(map));
}


PlannerStatus Planner::plan(const ob::ScopedState<>& start,
                            const ob::ScopedState<>& goal) {

  std::lock_guard<std::mutex> lock(map_mutex_);

  // Enforce goal pose inside bounds.
  auto space = ss_->getStateSpace();
  ob::ScopedState<> goal_clipped(goal);
  ob::ScopedState<> start_clipped(start);
  auto start_clipped_ptr = start_clipped->as<ob::SE3StateSpace::StateType>();
  auto goal_clipped_ptr = goal_clipped->as<ob::SE3StateSpace::StateType>();

  global_start_.clear();
  global_start_.push_back(start_clipped_ptr->getX());
  global_start_.push_back(start_clipped_ptr->getY());
  global_start_.push_back(start_clipped_ptr->getZ());
  global_start_.push_back(start_clipped_ptr->rotation().w);
  global_start_.push_back(start_clipped_ptr->rotation().x);
  global_start_.push_back(start_clipped_ptr->rotation().y);
  global_start_.push_back(start_clipped_ptr->rotation().z);

  if (!space_->satisfiesBounds(goal_clipped.get())) {
    if(params_->verbose) {
      printf("Original goal [%f\t%f\t%f], ",
             goal_clipped_ptr->getX(),
             goal_clipped_ptr->getY(),
             goal_clipped_ptr->getZ());
    }
    ss_->getStateSpace()->enforceBounds(goal_clipped.get());
    if (params_->verbose) {
      printf("clipped to [%f\t%f\t%f].\n",
             goal_clipped_ptr->getX(),
             goal_clipped_ptr->getY(),
             goal_clipped_ptr->getZ());
    }
  }

  // Get goal height from elevation map.
  if (map_->isInside(grid_map::Position(goal_clipped_ptr->getX(), goal_clipped_ptr->getY()))) {
    // Get height, roll, pitch from height map.
    grid_map::Position pos;
    pos.x() = goal_clipped_ptr->getX();
    pos.y() = goal_clipped_ptr->getY();
    const auto ind = map_->getIndexOfPosition(pos);
    goal_clipped_ptr->setZ(map_->getHeightAtIndex(ind));
    
    state_pos_.get()->values[2] = getYawFromSO3(goal_clipped_ptr->rotation());
    auto goal_yaw = state_pos_.get()->values[2];
    auto start_yaw = getYawFromSO3(start_clipped_ptr->rotation());
    auto diff_yaw = goal_yaw - start_yaw;
    if(std::abs(diff_yaw) > M_PI_2) state_pos_.get()->values[2] -= M_PI;
    

    const Eigen::Quaterniond R_wb(Eigen::AngleAxisd(state_pos_.get()->values[2], Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d normal_w = map_->getNormal(ind);
    const auto normal_b = R_wb.inverse() * normal_w;

    state_pos_.get()->values[0] = -atan2(normal_b.y(), normal_b.z()); // Roll.
    state_pos_.get()->values[1] = atan2(normal_b.x(), normal_b.z()); // Pitch.
    setSO3FromRPY(goal_clipped_ptr->rotation(), state_pos_.get()->values);
  }
  
  std::cout << "Setting start and goal points." << std::endl;
  // Reset planner and start planning.
  ss_->getPlanner()->clearQuery();
  ss_->setStartAndGoalStates(start, goal_clipped);

  std::cout << "Planning..." << std::endl;
  // attempt to solve the planning problem within one second of planning time
  ob::PlannerStatus solved;

  try {
    solved = ss_->solve(params_->planner.plan_time);
    solved_ = static_cast<bool>(solved);

  } catch (ompl::Exception& e) {
    // All graph edges to goal where actually invalid.
    std::cout << e.what() << std::endl;
    solved_ = false;
    return PlannerStatus::NOT_SOLVED;
  }

  switch (ob::PlannerStatus::StatusType(solved)) {
    case ob::PlannerStatus::INVALID_START: return PlannerStatus::INVALID_START;
    case ob::PlannerStatus::INVALID_GOAL: return PlannerStatus::INVALID_GOAL;
    case ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE: return PlannerStatus::INVALID_GOAL;
    case ob::PlannerStatus::TIMEOUT: return PlannerStatus::NOT_SOLVED;
    case ob::PlannerStatus::EXACT_SOLUTION: return PlannerStatus::SOLVED;
  }
  return PlannerStatus::UNKNOWN;
}


og::PathGeometric Planner::getSolutionPath(const bool& simplify) const {
  auto path = ss_->getSolutionPath();
  // path.interpolate();
  if (!solved_) {
    throw ompl::Exception("Requested failed solution path.");
  }
  if (simplify) {
    ss_->simplifySolution();
    const auto path_simple = ss_->getSolutionPath();

    // checkAndRepair() inside of simplifySolution might fail, but the result of that
    // operation is not accessible through SimpleSetup. That's why we need to do the
    // following checks to catch this case.
    if (!path_simple.check()) {
      std::cout << "Simplified path is invalid. Returning original." << std::endl;
    } else {
      const auto obj = ss_->getOptimizationObjective();
      const auto cost_simple = path_simple.cost(obj);
      const auto cost_orig = path.cost(obj);
      if (params_->verbose) {
        std::cout << "cost_simple " << cost_simple << std::endl;
        std::cout << "cost_orig " << cost_orig << std::endl;
      }
      if (obj->isCostBetterThan(cost_orig, cost_simple)) {
        if (params_->verbose) {
          std::cout << "Original path cost is lower than simplified. Returning original." << std::endl;
        }
      } else {
        path = path_simple;
      }
    }
  }
  return path;
}
