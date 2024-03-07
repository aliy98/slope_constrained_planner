/**
 * @file utils.cpp
 */

#include "slope_constrained_planner_ros/utils.h"


using namespace slope_constrained_planner;


ParamsPtr slope_constrained_planner::loadRosParameters(const ros::NodeHandle& nh) {
  ParamsPtr params = std::make_shared<Params>();

  // Planner.

  params->planner.name =
      getParamWithDefaultWarning(nh,
                                 "planner/name",
                                 params->planner.name);
  params->planner.elevation_layer =
      getParamWithDefaultWarning(nh,
                                 "planner/elevation_layer",
                                 params->planner.elevation_layer);
  params->planner.plan_time =
      getParamWithDefaultWarning(nh,
                                 "planner/plan_time",
                                 params->planner.plan_time);
  params->planner.n_threads =
      getParamWithDefaultWarning(nh,
                                 "planner/n_threads",
                                 params->planner.n_threads);
  params->planner.replan_freq =
      getParamWithDefaultWarning(nh,
                                 "planner/replan_freq",
                                 params->planner.replan_freq);

  params->planner.simplify_solution =
      getParamWithDefaultWarning(nh,
                                 "planner/simplify_solution",
                                 params->planner.simplify_solution);
  params->planner.snap_goal_to_map =
      getParamWithDefaultWarning(nh,
                                 "planner/snap_goal_to_map",
                                 params->planner.snap_goal_to_map);


  // Sampler.

  params->sampler.max_pitch_pert =
      getParamWithDefaultWarning(nh,
                                 "sampler/max_pitch_pert",
                                 params->sampler.max_pitch_pert);
  params->sampler.max_roll_pert =
      getParamWithDefaultWarning(nh,
                                 "sampler/max_roll_pert",
                                 params->sampler.max_roll_pert);
  params->sampler.sample_from_distribution =
      getParamWithDefaultWarning(nh,
                                 "sampler/sample_from_distribution",
                                 params->sampler.sample_from_distribution);
  params->sampler.use_inverse_vertex_density =
      getParamWithDefaultWarning(nh,
                                 "sampler/use_inverse_vertex_density",
                                 params->sampler.use_inverse_vertex_density);
  params->sampler.use_max_prob_unknown_samples =
      getParamWithDefaultWarning(nh,
                                 "sampler/use_max_prob_unknown_samples",
                                 params->sampler.use_max_prob_unknown_samples);
  params->sampler.max_prob_unknown_samples =
      getParamWithDefaultWarning(nh,
                                 "sampler/max_prob_unknown_samples",
                                 params->sampler.max_prob_unknown_samples);
  params->sampler.reach_min_phi =
      getParamWithDefaultWarning(nh,
                                 "sampler/reach_min_phi",
                                 params->sampler.reach_min_phi);
  params->sampler.consider_theta =
      getParamWithDefaultWarning(nh,
                                 "sampler/consider_theta",
                                 params->sampler.consider_theta);
  // Convert degrees to rad.
  params->sampler.max_pitch_pert *= M_PI / 180;
  params->sampler.max_roll_pert *= M_PI / 180;


  // Validator
  params->validator.max_motion_slope =
      getParamWithDefaultWarning(nh,
                                 "validator/max_motion_slope",
                                 params->validator.max_motion_slope);

  params->validator.max_motion_length =
      getParamWithDefaultWarning(nh,
                                 "validator/max_motion_length",
                                 params->validator.max_motion_length);

  params->validator.min_motion_length =
      getParamWithDefaultWarning(nh,
                                 "validator/min_motion_length",
                                 params->validator.min_motion_length);

  params->validator.max_motion_yaw_diff =
      getParamWithDefaultWarning(nh,
                                 "validator/max_motion_yaw_diff",
                                 params->validator.max_motion_yaw_diff);

  params->validator.max_motion_slope_yaw_diff_free =
      getParamWithDefaultWarning(nh,
                                 "validator/max_motion_slope_yaw_diff_free",
                                 params->validator.max_motion_slope_yaw_diff_free);

  params->validator.max_state_phi =
      getParamWithDefaultWarning(nh,
                                 "validator/max_state_phi",
                                 params->validator.max_state_phi);
  params->validator.max_state_theta =
      getParamWithDefaultWarning(nh,
                                 "validator/max_state_theta",
                                 params->validator.max_state_theta);


  // Robot.

  params->robot.base_frame =
      getParamWithDefaultWarning(nh,
                                 "robot/base_frame",
                                 params->robot.base_frame);

  // Robot / Torso.

  params->robot.torso.length =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/length",
                                 params->robot.torso.length);
  params->robot.torso.width =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/width",
                                 params->robot.torso.width);
  params->robot.torso.height =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/height",
                                 params->robot.torso.height);

  // Robot / Torso / Offset.

  params->robot.torso.offset.x =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/offset/x",
                                 params->robot.torso.offset.x);
  params->robot.torso.offset.y =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/offset/y",
                                 params->robot.torso.offset.y);
  params->robot.torso.offset.z =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/offset/z",
                                 params->robot.torso.offset.z);

  // Robot / Feet / Offset.

  params->robot.feet.offset.x =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/offset/x",
                                 params->robot.feet.offset.x);
  params->robot.feet.offset.y =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/offset/y",
                                 params->robot.feet.offset.y);
  params->robot.feet.offset.z =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/offset/z",
                                 params->robot.feet.offset.z);

  // Robot / Feet / Reach.

  params->robot.feet.reach.x =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/reach/x",
                                 params->robot.feet.reach.x);
  params->robot.feet.reach.y =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/reach/y",
                                 params->robot.feet.reach.y);
  params->robot.feet.reach.z =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/reach/z",
                                 params->robot.feet.reach.z);

  params->verbose = getParamWithDefaultWarning(nh, "verbose", false);

  return params;
}
