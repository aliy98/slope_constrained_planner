/**
 * @file params.h
 * @brief The defined parameters to be used in different components of the software architecture
 */

#pragma once

#include <string>
#include <cmath>
#include <memory>


namespace slope_constrained_planner{


// For parameter description see "params.yaml".
struct Params {

  struct {
    std::string    name{"prm_star"};
    std::string    elevation_layer{"elevation"};
    double         plan_time{1.0};
    unsigned int   n_threads{1};
    double         replan_freq{0.5};
    bool           simplify_solution{false};
    bool           snap_goal_to_map{true};

  } planner;

  struct {
    double         max_pitch_pert{10.0 / 180*M_PI};
    double         max_roll_pert{3.33 / 180*M_PI};
    bool           sample_from_distribution{true};
    bool           use_inverse_vertex_density{false};
    bool           use_max_prob_unknown_samples{false};
    double         max_prob_unknown_samples{0.1};
    bool           reach_min_phi{false};
    bool           consider_theta{false};
  } sampler;

  struct {
    double         max_motion_slope{15.0};
    double         max_motion_length{2.0};
    double         min_motion_length{1.0};
    double         max_motion_yaw_diff{30.0};
    double         max_motion_slope_yaw_diff_free{5.0};
    double         max_state_phi{10.0};
    double         max_state_theta{10.0};
  } validator;

  struct {
    std::string    base_frame{"base_link"};

    struct {
      double         length{1.05};
      double         width{0.55};
      double         height{0.2};

      struct {
        double         x{0.0};
        double         y{0.0};
        double         z{0.0};
      } offset;

    } torso;

    struct {

      struct {
        double         x{0.362};
        double         y{0.225};
        double         z{-0.525};
      } offset;

      struct {
        double         x{0.25};
        double         y{0.1};
        double         z{0.15};
      } reach;

    } feet;

  } robot;

  bool verbose{true};

};



using ParamsPtr = std::shared_ptr<Params>;
using ParamsConstPtr = std::shared_ptr<const Params>;



}
