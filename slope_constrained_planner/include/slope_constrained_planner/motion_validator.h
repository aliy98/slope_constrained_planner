/**
 * @file motion_validator.h
 * @brief Custom motion validator class which checks the slope of motion between two states
 */

#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <slope_constrained_planner/utils.h>
#include <slope_constrained_planner/params.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @namespace slope_constrained_planner
*/
namespace slope_constrained_planner {

/**
 * @brief Custom motion validator class which checks the slope of motion between two states
*/
class SlopeConstrainedMotionValidator : public ob::MotionValidator {

    protected:
        /// Parameters containing the data for map layers
        ParamsConstPtr params_;

    public:

        /**
         * @brief Constructor 
         * @param si space information of the state space
        */
        SlopeConstrainedMotionValidator(const ob::SpaceInformationPtr &si,
                                        const ParamsConstPtr& params);

        /**
         * @brief Checks the slope of the motion using the first and second states position, if the slope
         * is below the threshold, it returns true otherwise it would return false
         * @param s1 First state in the motion
         * @param s2 Second state in the motion
         * @return validity of motion
        */
        bool checkMotion(const ob::State* s1, const ob::State* s2) const override;

        /**
         * @brief Checks the slope of the motion using the first and second states position, if the slope
         * is below the threshold, it returns true otherwise it would return false
         * @param s1 First state in the motion
         * @param s2 Second state in the motion
         * @return validity of motion
        */
        bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override;
};

}
