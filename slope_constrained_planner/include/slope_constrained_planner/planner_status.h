/**
 * @file planner_status.h
 * @brief Defines the different status for the planner
 */

#pragma once


namespace slope_constrained_planner {


enum PlannerStatus {
  UNKNOWN = 0,
  INVALID_START,
  INVALID_GOAL,
  NO_MAP,
  NOT_SOLVED,
  SOLVED
};


}
