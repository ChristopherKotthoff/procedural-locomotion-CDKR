//
// Created by Dongho Kang on 02.05.23.
//

#ifndef PROCEDURAL_LOCOMOTION_GAITPLANNER_H
#define PROCEDURAL_LOCOMOTION_GAITPLANNER_H

#include "loco/planner/FootFallPattern.h"

namespace crl::loco {

/**
 * base class
 */
class GaitPlanner {
public:
    virtual ~GaitPlanner() = default;

    virtual PeriodicGait getPeriodicGait(const std::shared_ptr<LeggedRobot> &robot) const = 0;
};

/**
 * for quadrupedal characters
 */
class QuadrupedalGaitPlanner : public GaitPlanner {
public:
    ~QuadrupedalGaitPlanner() override = default;

    PeriodicGait getPeriodicGait(const std::shared_ptr<LeggedRobot> &robot) const {
        PeriodicGait pg;
        double tOffset = -0.0;
        pg.addSwingPhaseForLimb(robot->getLimbByName("hl"), 0 - tOffset, 0.5 + tOffset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("fl"), 0.5 - tOffset, 1.0 + tOffset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("hr"), 0.5 - tOffset, 1.0 + tOffset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("fr"), 0 - tOffset, 0.5 + tOffset);
        pg.strideDuration = 0.5;
        return pg;
    }
};

/**
 * for bipedal characters
 */
class BipedalGaitPlanner : public GaitPlanner {
public:
    ~BipedalGaitPlanner() override = default;

    PeriodicGait getPeriodicGait(const std::shared_ptr<LeggedRobot> &robot) const {
        PeriodicGait pg;
        double offset = -0.1;
        pg.addSwingPhaseForLimb(robot->getLimbByName("lLowerLeg"), 0 - 0.13, 0.5 - offset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("rLowerLeg"), 0.5 - 0.13, 1.0 - offset);
        pg.addSwingPhaseForLimb(robot->getLimbByName("lHand"), 0.0, 0.999);
        pg.addSwingPhaseForLimb(robot->getLimbByName("rHand"), -0.5, 0.499);
        pg.addSwingPhaseForLimb(robot->getLimbByName("head"), 0.0, 0.999); // For a non foot limb, we should set the swing phase to 0.0 to 1.0
        pg.addSwingPhaseForLimb(robot->getLimbByName("pelvis"), 0.0, 0.999); // For a non foot limb, we should set the swing phase to 0.0 to 1.0
        pg.strideDuration = 0.8;
        return pg;
    }
};

}  // namespace crl::loco

#endif  //PROCEDURAL_LOCOMOTION_GAITPLANNER_H
