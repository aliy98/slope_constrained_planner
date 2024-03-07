/**
 * @file sampler.cpp
 * @brief Contains the definition of public member functions of `Sampler` class
 */

#include "slope_constrained_planner/sampler.h"
#include "slope_constrained_planner/utils.h"


using namespace slope_constrained_planner;


Sampler::Sampler(const ob::StateSpace *space,
                                     const std::shared_ptr<Map> &map,
                                     const ParamsConstPtr &params)
    : ob::StateSampler(space),
      params_(params),
      space_real_vec_(new ob::RealVectorStateSpace(3)),
      space_rot_(new ob::SE3StateSpace()),
      bounds_se3_(new ob::RealVectorBounds(3)),
      state_pos_(space_real_vec_),
      state_rot_(space_rot_),
      base_real_vec_(space_real_vec_.get()),
      map_(map) {
        
  // Set SE2 bounds from SE3.
  auto bounds_se3 = space_->as<ob::SE3StateSpace>()->getBounds();
  bounds_se3_->setLow(0, bounds_se3.low[0]);
  bounds_se3_->setLow(1, bounds_se3.low[1]);
  bounds_se3_->setLow(2, bounds_se3.low[2]);
  bounds_se3_->setHigh(0, bounds_se3.high[0]);
  bounds_se3_->setHigh(1, bounds_se3.high[1]);
  bounds_se3_->setHigh(2, bounds_se3.high[2]);

  std::vector<std::string> layers = map_->getLayers();
  for (std::string i: layers)
    std::cout << i << ' ';
  std::cout << std::endl;

  std::cout << "Lower SE3 position bounds: " << bounds_se3.low[0] << "\t"
                                            << bounds_se3.low[1] << "\t"
                                            << bounds_se3.low[2] << std::endl;
  std::cout << "Upper SE3 position bounds: " << bounds_se3.high[0] << "\t"
                                            << bounds_se3.high[1] << "\t"
                                            << bounds_se3.high[2] << std::endl;
  space_real_vec_->as<ob::RealVectorStateSpace>()->setBounds(bounds_se3);
}


bool Sampler::checkSampleDistance(const ob::ScopedState<>& sample) {
  auto sample_ptr = sample->as<ob::RealVectorStateSpace::StateType>();
  auto x1 = sample_ptr->values[0];
  auto y1 = sample_ptr->values[1];
  auto x2 = global_start_[0];
  auto y2 = global_start_[1];
  auto dist = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  if (dist < params_->robot.torso.length) return true;
  else return false;
}

grid_map::Position Sampler::samplePositionInMap() {
  grid_map::Position pos;
  grid_map::Index ind;
  double normal = 0;
  do {
  do {
    base_real_vec_.sampleUniform(state_pos_.get());
    pos.x() = state_pos_.get()->values[0];
    pos.y() = state_pos_.get()->values[1];
    sample_is_near_robot_ = checkSampleDistance(state_pos_);
  } while (!map_->isInside(pos) && !sample_is_near_robot_);
  if (!sample_is_near_robot_) {
    ind = map_->getIndexOfPosition(pos);
    normal = map_->getNormal(ind).mean();
  } else 
    normal = 1;
  } while (normal == 0);

  return pos;
}


void Sampler::sampleUniform(ob::State* state) {
  auto state_se3 = state->as<ob::SE3StateSpace::StateType>();

  std::lock_guard<std::mutex> lock(map_mutex_);
  Eigen::Vector3d i_s(1, 0, 0);
  Eigen::Vector3d j_s(0, 1, 0);

  if (!prev_state_is_valid_) {

    // Definition of required variables
    bool state_found = false;
    bool skip_state = false;
    sample_is_near_robot_ = false;
    double phi;
    double theta;
    double yaw;
    while (!state_found && !sample_is_near_robot_) {

      // Sample new position in map
      // std::cout<<"Sampling new state position..."<<std::endl;
      grid_map::Position pos;
      pos = samplePositionInMap();
      if (!sample_is_near_robot_) {
        // std::cout<<"is not near"<<std::endl;
        prev_pos_ = pos;
        const auto ind = map_->getIndexOfPosition(pos);
        state_pos_.get()->values[0] = pos.x();
        state_pos_.get()->values[1] = pos.y();
        state_pos_.get()->values[2] = map_->getHeightAtIndex(ind); // Get z component from map
        Eigen::Vector3d normal_w = map_->getNormal(ind);

        // Set position in the state
        state_se3->setX(state_pos_.get()->values[0]);
        state_se3->setY(state_pos_.get()->values[1]);
        state_se3->setZ(state_pos_.get()->values[2]);

        // Sample rotation for the found state position
        state_pos_.get()->values[2] = rng_.uniformReal(-M_PI, M_PI); // Find yaw angle randomly
        double init_yaw = state_pos_.get()->values[2] * 180 / M_PI;

        // Rotate state frame along z axis to find the desired phi
        // std::cout<<"Rotating the sample frame..."<<std::endl;
        bool yaw_passed_pi = false;
        do {
          // Get roll and pitch values from state cell in the map
          yaw = state_pos_.get()->values[2] * 180 / M_PI;
          Eigen::Quaterniond R_wb(Eigen::AngleAxisd(state_pos_.get()->values[2], Eigen::Vector3d::UnitZ()));
          auto normal_b = R_wb.inverse() * normal_w;
          state_pos_.get()->values[0] = -atan2(normal_b.y(), normal_b.z()); // Roll.
          state_pos_.get()->values[1] = atan2(normal_b.x(), normal_b.z()); // Pitch.

          // Get rotation matrix between <w> and <s>
          setSO3FromRPY(state_rot_->rotation(), state_pos_.get()->values);
          double q0 = state_rot_->rotation().w;
          double q1 = state_rot_->rotation().x;
          double q2 = state_rot_->rotation().y;
          double q3 = state_rot_->rotation().z;
          Eigen::Quaterniond R_ws(q0, q1, q2, q3);

          // Get phi value
          auto i_w = R_ws* i_s;
          phi = atan(i_w[2] / sqrt(i_w[0]*i_w[0] + i_w[1]*i_w[1]));
          phi = phi * 180 / M_PI;
          // std::cout<<"phi: "<<phi<<std::endl;

          // Get theta value
          auto j_w = R_ws* j_s;
          theta = atan(j_w[2] / sqrt(j_w[0]*j_w[0] + j_w[1]*j_w[1]));
          theta = theta * 180 / M_PI;
          // std::cout<<"theta: "<<theta<<std::endl;

          if (abs(phi) > params_->validator.max_state_phi || 
              (abs(theta) > params_->validator.max_state_theta && params_->sampler.consider_theta)) {
          if (yaw > 0 && yaw <= 180) state_pos_.get()->values[2] -= 0.05;
          if (yaw < 0 && yaw >= -180) state_pos_.get()->values[2] -= 0.05;
          if (yaw < -180) {
            state_pos_.get()->values[2] = M_PI;
            yaw_passed_pi = true;
          }
          if (abs(yaw - init_yaw) < 5 && yaw_passed_pi) skip_state = true;
          } else {
            state_found = true;
            skip_state = true;
          }
        } while (!skip_state);

        prev_state_is_valid_ = true;
        prev_yaw_ = state_pos_.get()->values[2];
        prev_phi_ = phi;

      } else {   // Sample is near robot 

        // std::cout<<"is near"<<std::endl;
        state_pos_.get()->values[0] = pos.x();
        state_pos_.get()->values[1] = pos.y();
        state_pos_.get()->values[2] = global_start_[2];

        // Set position in the state
        state_se3->setX(state_pos_.get()->values[0]);
        state_se3->setY(state_pos_.get()->values[1]);
        state_se3->setZ(state_pos_.get()->values[2]);

        state_pos_.get()->values[0] = getRollFromQuat(global_start_[3], global_start_[4],
                                                      global_start_[5], global_start_[6]);
        state_pos_.get()->values[1] = getPitchFromQuat(global_start_[3], global_start_[4],
                                                      global_start_[5], global_start_[6]);
        state_pos_.get()->values[2] = getYawFromQuat(global_start_[3], global_start_[4],
                                                    global_start_[5], global_start_[6]);
      }
    }

    // Set orientation in the state
    setSO3FromRPY(state_se3->rotation(), state_pos_.get()->values);

  } else {   // Considering the case when prev state was valid

    prev_state_is_valid_ = false;

    // Use previously found position for sampling
    const auto ind = map_->getIndexOfPosition(prev_pos_);
    state_pos_.get()->values[0] = prev_pos_.x();
    state_pos_.get()->values[1] = prev_pos_.y();
    state_pos_.get()->values[2] = map_->getHeightAtIndex(ind); // Get z component from map
    Eigen::Vector3d normal_w = map_->getNormal(ind);

    // Set previous position in the state
    state_se3->setX(state_pos_.get()->values[0]);
    state_se3->setY(state_pos_.get()->values[1]);
    state_se3->setZ(state_pos_.get()->values[2]);

    // Sample orientation
    state_pos_.get()->values[2] = prev_yaw_ - M_PI; // Use prevoius yaw angle - pi

    // Definition of required variables
    double phi;
    double yaw;

    // std::cout<<"Using previously found state with reversed orientation..."<<std::endl;

    // Get roll and pitch values from state cell in the map
    yaw = state_pos_.get()->values[2] * 180 / M_PI;
    Eigen::Quaterniond R_wb(Eigen::AngleAxisd(state_pos_.get()->values[2], Eigen::Vector3d::UnitZ()));
    auto normal_b = R_wb.inverse() * normal_w;
    state_pos_.get()->values[0] = -atan2(normal_b.y(), normal_b.z()); // Roll.
    state_pos_.get()->values[1] = atan2(normal_b.x(), normal_b.z()); // Pitch.

    // Get rotation matrix between <w> and <s>
    setSO3FromRPY(state_rot_->rotation(), state_pos_.get()->values);
    double q0 = state_rot_->rotation().w;
    double q1 = state_rot_->rotation().x;
    double q2 = state_rot_->rotation().y;
    double q3 = state_rot_->rotation().z;
    Eigen::Quaterniond R_ws(q0, q1, q2, q3);

    // Get phi
    auto i_w = R_ws* i_s;
    phi = atan(i_w[2] / sqrt(i_w[0]*i_w[0] + i_w[1]*i_w[1]));
    phi = phi * 180 / M_PI;
    // std::cout<<"phi: "<<phi<<std::endl;
    // std::cout<<"prev phi: "<<prev_phi_<<std::endl;

    // Set orientation in the state
    setSO3FromRPY(state_se3->rotation(), state_pos_.get()->values);
  }
}


void Sampler::sampleUniformNear(ob::State* /*state*/,
                                          const ob::State* /*near*/,
                                          double /*distance*/) {
  throw ompl::Exception("MyValidStateSampler::sampleUniformNear", "not implemented");
}


void Sampler::sampleGaussian(ob::State* /*state*/,
                                       const ob::State* /*mean*/,
                                       double /*std_dev*/) {
  throw ompl::Exception("MyValidStateSampler::sampleGaussian", "not implemented");
}


void SamplerAllocator::setMap(const std::shared_ptr<Map> &map) {
  map_ = map;
}


std::shared_ptr<Sampler> SamplerAllocator::getSampler(const ob::StateSpace* space) {
  return std::make_shared<Sampler>(space, map_, params_);
}
