/**
 * @file motion_validator.cpp
 * @brief Contains the definition of public member functions of `MotionValidator` class
 */

#include <slope_constrained_planner/motion_validator.h>

using namespace slope_constrained_planner;

SlopeConstrainedMotionValidator::SlopeConstrainedMotionValidator(const ob::SpaceInformationPtr &si,
                                                                 const ParamsConstPtr& params)
  : ob::MotionValidator(si),
    params_(params) {
    }

bool SlopeConstrainedMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const {

    const ob::SE3StateSpace::StateType* s1_se3 = s1->as<ob::SE3StateSpace::StateType>();    
    const ob::SE3StateSpace::StateType* s2_se3 = s2->as<ob::SE3StateSpace::StateType>();

    double x1 = s1_se3->getX();
    double x2 = s2_se3->getX();
    double y1 = s1_se3->getY();
    double y2 = s2_se3->getY();
    double z1 = s1_se3->getZ();
    double z2 = s2_se3->getZ();

    double diff_x = x2 - x1;
    double diff_y = y2 - y1;
    double diff_z = z2 - z1;

    double phi = atan(diff_z / sqrt(diff_x*diff_x + diff_y*diff_y));

    auto yaw1 = getYawFromSO3(s1_se3->rotation());
    auto yaw2 = getYawFromSO3(s2_se3->rotation());

    auto roll1 = getRollFromSO3(s1_se3->rotation());
    auto roll2 = getRollFromSO3(s2_se3->rotation());

    auto pitch1 = getPitchFromSO3(s1_se3->rotation());
    auto pitch2 = getPitchFromSO3(s2_se3->rotation());

    auto delta = params_->validator.max_motion_slope_yaw_diff_free;
    
    if (abs(roll1) < delta && abs(roll2) < delta && abs(pitch1) < delta && abs(pitch2) < delta) {
        if (abs(phi) * 180 / M_PI > params_->validator.max_motion_slope 
        || sqrt(diff_x*diff_x + diff_y*diff_y) > params_->validator.max_motion_length 
        || sqrt(diff_x*diff_x + diff_y*diff_y) < params_->validator.min_motion_length) 
            return false;
        else
            return true;
    } else {
        if (abs(phi) * 180 / M_PI > params_->validator.max_motion_slope 
        || sqrt(diff_x*diff_x + diff_y*diff_y) > params_->validator.max_motion_length 
        || sqrt(diff_x*diff_x + diff_y*diff_y) < params_->validator.min_motion_length
        || abs(yaw2 - yaw1) * 180 / M_PI > params_->validator.max_motion_yaw_diff) 
            return false;
        else
            return true;
    }
}

bool SlopeConstrainedMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const {
    // std::cout<<"TRUE"<<std::endl;
    return false;
}


