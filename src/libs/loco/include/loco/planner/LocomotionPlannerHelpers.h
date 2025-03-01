#pragma once

#include <crl-basic/gui/renderer.h>
#include <crl-basic/utils/trajectory.h>
#include <loco/planner/BodyFrame.h>
#include <loco/planner/FootFallPattern.h>
#include <loco/planner/LocomotionTrajectoryPlanner.h>
#include <loco/robot/RB.h>
#include <loco/robot/RBJoint.h>
#include <loco/robot/RBUtils.h>
#include "math.h"
#include <loco/shared/value_share.h>

namespace crl::loco {

class LimbMotionProperties {
public:
    double contactSafetyFactor = 0.0;

    // p: this trajectory controlls how fast a foot lifts and how fast it sets
    // back down, encoded as a function of swing phase
    Trajectory1D swingFootHeightTraj;
    Trajectory3D generalSwingTraj;
    // p: given the total step length for a limb, ffStepLengthRatio controls the
    // stance phase when the limb should be right below the hip/shoulder (e.g.
    // default, or zero step length configuration) Should this be per limb?
    double ffStancePhaseForDefaultStepLength = 0.4;

    // to account for a non-zero foot size, we add an offset to the swing foot
    // height. This will allow us to control how aggressively the robot steps
    // down, and it will allow firm contacts to be established
    Trajectory1D swingHeightOffsetTrajDueToFootSize;

    double swingFootHeight = 1;

    //x and z here are expressed in a heading-independent coordinate frame
    double stepWidthOffsetX = 0.7;
    double stepWidthOffsetZ = 1.0;

    /*
    * constructor: no arguments, for backward-compatibility
    */
    LimbMotionProperties() {
        // p: this trajectory should be parameterized...
        swingFootHeightTraj.addKnot(0, 0);
        swingFootHeightTraj.addKnot(0.5, 0.3);
        swingFootHeightTraj.addKnot(1.0, 0);

        swingHeightOffsetTrajDueToFootSize.addKnot(0, 1.0);
        swingHeightOffsetTrajDueToFootSize.addKnot(0.5, 1.0);
        // when the transition from swing to stance happens, in order to make
        // sure a firm contact is established, the contactSafetyFactor(default = 0.7)
        //here makes the contact be pretty firm.
        swingHeightOffsetTrajDueToFootSize.addKnot(1.0, contactSafetyFactor);
    }

    /*
    * constructor: based on limb
    */
    LimbMotionProperties(std::shared_ptr<RobotLimb> limb) {
        bool is_leg = limb->name == "lLowerLeg" || limb->name == "rLowerLeg";
        bool is_foot = limb->name == "lToes" || limb->name == "rToes";
        bool is_hand = limb->name == "lHand" || limb->name == "rHand";
        bool is_head = limb->name == "head";
        bool is_pelvis = limb->name == "pelvis";
        limb->normalizedSpeed = 1.0;
        if (targetForwardSpeed_shared != NULL){
            limb->normalizedSpeed = std::clamp(*targetForwardSpeed_shared, 0.0, maxSpeed) / maxSpeed; // We should also allow negative speeds.
        }
        double speed = limb->normalizedSpeed;
            
        if (is_leg) {
            // p: this trajectory should be parameterized...
            generalSwingTraj.addKnot(0, V3D(0, 0, 0.0));
            generalSwingTraj.addKnot(0.2, V3D(0, 0.1 + speed * 0.5, speed * -0.15));
            generalSwingTraj.addKnot(0.6, V3D(0, 0.1 + speed * 0.6, 0.0));
            generalSwingTraj.addKnot(0.8, V3D(0, 0.1 + speed * 0.3, speed * 0.1));
            generalSwingTraj.addKnot(1.0, V3D(0, 0, 0.0));


            swingHeightOffsetTrajDueToFootSize.addKnot(0, 0.0);
            swingHeightOffsetTrajDueToFootSize.addKnot(0.5, 0.0);
            // when the transition from swing to stance happens, in order to make
            // sure a firm contact is established, the contactSafetyFactor(default = 0.7)
            //here makes the contact be pretty firm.
            swingHeightOffsetTrajDueToFootSize.addKnot(1.0, 0.0);
        } else if (is_foot) {
            // p: this trajectory should be parameterized...
            generalSwingTraj.addKnot(0, V3D(0, 0, 0.0));
            generalSwingTraj.addKnot(0.2, V3D(0, speed * 0.5, speed * -0.15));
            generalSwingTraj.addKnot(0.6, V3D(0, speed * 0.6, 0.0));
            generalSwingTraj.addKnot(0.8, V3D(0, speed * 0.3, speed * 0.1));
            generalSwingTraj.addKnot(1.0, V3D(0, 0, 0.0));


            swingHeightOffsetTrajDueToFootSize.addKnot(0, 0.0);
            swingHeightOffsetTrajDueToFootSize.addKnot(0.5, 0.0);
            // when the transition from swing to stance happens, in order to make
            // sure a firm contact is established, the contactSafetyFactor(default = 0.7)
            //here makes the contact be pretty firm.
            swingHeightOffsetTrajDueToFootSize.addKnot(1.0, 0.0);
        } else if (is_hand) {
            double yMaxFor = 0.0;
            double zMaxFor = 0.0;
            double zMaxBack = 0.0;
            double yMaxBack = 0.0;
            double yMinMid = 0.0;
            double xHandIn = 0.0;
            if (speed != 0.0) {
                yMaxFor = 0.05 + speed * 0.6;
                zMaxFor = 0.2 + speed * 0.1;
                zMaxBack = 0.2 * speed + 0.1;
                yMaxBack = 0.05 + speed * 0.1;
                yMinMid = 0.2 * speed;
                xHandIn = speed * 0.1;

            }
            if (limb->name == "lHand") { // Bit ugly, but both hands need to face inwards.
                xHandIn = -xHandIn;
            }

            generalSwingTraj.addKnot(0, V3D(0, yMinMid + (yMaxFor - yMinMid) / 2, 0.9 * zMaxFor));  // Meet in the middle
            generalSwingTraj.addKnot(0.125, V3D(xHandIn, yMaxFor, zMaxFor));
            generalSwingTraj.addKnot(0.25, V3D(0, yMinMid, 0.25 * zMaxFor));
            generalSwingTraj.addKnot(0.625, V3D(0, yMaxBack, -zMaxBack));
            generalSwingTraj.addKnot(0.75, V3D(0, yMinMid, 0.5 * zMaxFor));
            generalSwingTraj.addKnot(1.0, V3D(0, yMinMid + (yMaxFor - yMinMid) / 2, 0.9 * zMaxFor));
        } else if (is_head) {
            // p: this trajectory should be parameterized...
            double headBop = 0.01;
            double headLeanForward = speed > (walkToRunTransitionSpeed / maxSpeed) ? speed * 0.3 : 0.0;
            generalSwingTraj.addKnot(0, V3D(0, 0, headLeanForward));
            generalSwingTraj.addKnot(0.125, V3D(0, headBop, headLeanForward));
            generalSwingTraj.addKnot(0.375, V3D(0, -headBop, headLeanForward));
            generalSwingTraj.addKnot(0.635, V3D(0, headBop, headLeanForward));
            generalSwingTraj.addKnot(0.875, V3D(0, -headBop, headLeanForward));
            generalSwingTraj.addKnot(1.0, V3D(0, 0, headLeanForward));
        } else if (is_pelvis) {
            double pelvisBop = 0.05 + speed * 0.1;
            generalSwingTraj.addKnot(0, V3D(0, -0.5 * pelvisBop, 0));
            generalSwingTraj.addKnot(0.125, V3D(0, -pelvisBop, 0));
            generalSwingTraj.addKnot(0.375, V3D(0, 0, 0));
            generalSwingTraj.addKnot(0.625, V3D(0, -pelvisBop, 0));
            generalSwingTraj.addKnot(0.875, V3D(0, 0, 0));
            generalSwingTraj.addKnot(1.0, V3D(0, -0.5*pelvisBop, 0));
        } else {
            assert(false && "LimbMotionProperties: unknown limb type");
        }
    }
};

class PlannedLimbContact {
public:
    //the time the contact phase starts
    double tStart = 0;
    //the time it ends
    double tEnd = 0;
    //and the position where the limb is/will be in contact with the environment
    P3D contactLocation;
    //we will also add a flag here that tells us if this is a current position (i.e. fixed), or a planned one
    bool isFixed = false;
    PlannedLimbContact(double tStart, double tEnd, const P3D& pos, bool isFixed = false) {
        this->tStart = tStart;
        this->tEnd = tEnd;
        this->contactLocation = pos;
        this->isFixed = isFixed;
    }
};

/**
        This class stores a sequence of planned foot falls for each of the robot's limbs.
        Useful bits of information, such as the target stepping location at a particular
        moment in time can be easily retrieved.
    */
class FootstepPlan {
public:
    //this is a pointer to the contact plan manager which corresponds to the sequnce of foot steps that is planned here.
    //Whomever populates the footstep plan should also set the contact plan manager adequately;
    //the two are closely link, as they store complementary information
    ContactPlanManager* cpm = nullptr;
    std::map<const std::shared_ptr<RobotLimb>, DynamicArray<PlannedLimbContact>> footSteps;
    crl::gui::SizeableGroundModel ground = crl::gui::SizeableGroundModel(10);

    //if the limb is in stance at time t, we'll be returning the current planned
    //position of the contact point; if the limb is in swing at time t, we'll
    //be returning the planned contact position for the end of the swing phase
    P3D getCurrentOrUpcomingPlannedContactLocation(const std::shared_ptr<RobotLimb>& limb, double t) {
        for (uint i = 0; i < footSteps[limb].size(); i++) {
            if (t < footSteps[limb][i].tEnd)
                return footSteps[limb][i].contactLocation;
        }
        return P3D();
    }

    PlannedLimbContact* getCurrentOrUpcomingPlannedLimbContact(const std::shared_ptr<RobotLimb>& limb, double t) {
        for (uint i = 0; i < footSteps[limb].size(); i++) {
            if (t < footSteps[limb][i].tEnd)
                return &footSteps[limb][i];
        }
        return nullptr;
    }

    int getIndexOfCurrentOrUpcomingPlannedLimbContact(const std::shared_ptr<RobotLimb>& limb, double t) {
        for (uint i = 0; i < footSteps[limb].size(); i++) {
            if (t < footSteps[limb][i].tEnd)
                return i;
        }

        return -1;
    }

    Trajectory3D generateNonFootTrajectory(
        const std::shared_ptr<RobotLimb> limb,
        const LimbMotionProperties& lmp,
        double tStart,
        double tEnd,
        double dt,
        Trajectory3D bFramePosTrajectory,
        Trajectory1D bFrameHeadingTrajectory
    ) {
        double t = tStart;
        V3D startingEEPos = V3D(limb->getEEWorldPos());
        Trajectory3D traj;
        traj.addKnot(t, startingEEPos);
        t += dt;
        while (t < tEnd) {
            double bFrameHeadingAngle = bFrameHeadingTrajectory.evaluate_catmull_rom(t);
            P3D bFramePos = P3D() + bFramePosTrajectory.evaluate_catmull_rom(t);
            V3D defaultEEOffset = limb->defaultEEOffset;
            ContactPhaseInfo cpiSwing = cpm->getCPInformationFor(limb, t);
            P3D pos = bFramePos + 
                getRotationQuaternion(bFrameHeadingAngle, V3D(0, 1, 0)) * defaultEEOffset + 
                getRotationQuaternion(bFrameHeadingAngle, V3D(0, 1, 0)) * lmp.generalSwingTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed());
            traj.addKnot(t, V3D(pos));
            t += dt;
        }
        return traj;
    }

    Trajectory3D generateNonFootTrajectory(
        const std::shared_ptr<LeggedRobot> robot,
        int limbIndex,
        const LimbMotionProperties& lmp,
        double tStart,
        double tEnd,
        double dt,
        Trajectory3D bFramePosTrajectory,
        Trajectory1D bFrameHeadingTrajectory
    ) {
        return generateNonFootTrajectory(robot->getLimb(limbIndex), lmp, tStart, tEnd, dt, bFramePosTrajectory, bFrameHeadingTrajectory);
    }

    //given world coordinates for the step locations, generate continuous trajectories for each of a robot's feet
    Trajectory3D generateLimbTrajectory(
        const std::shared_ptr<LeggedRobot> robot,
        int limbIndex,
        const LimbMotionProperties& lmp,
        double tStart,
        double tEnd,
        double dt,
        Trajectory3D bFramePosTrajectory,
        Trajectory1D bFrameHeadingTrajectory
    ) {
        const std::shared_ptr<RobotLimb>& limb = robot->getLimb(limbIndex);

        // Print name of the limb
        double t = tStart;
        Trajectory3D traj;
        Trajectory3D rawTraj; // Displacements are added onto this.

        //always start the trajectory from the current location of the robot
        V3D startingEEPos = V3D(limb->getEEWorldPos());
        traj.addKnot(t, startingEEPos);
        rawTraj.addKnot(t, startingEEPos);

        t += dt;
        while (t < tEnd) {
            ContactPhaseInfo cpi = cpm->getCPInformationFor(limb, t);
            if (cpi.isStance()) {
                double tEndOfStance = t + cpi.getTimeLeft();
                // in stance, we want the foot to not slip, while keeping to
                // the ground...
                V3D eePos = traj.getKnotValue(traj.getKnotCount() - 1);
                double groundHeight = ground.getHeight(eePos[0], eePos[2]); //  + offset;
                eePos.y() = groundHeight + limb->ee->radius * lmp.contactSafetyFactor;  // account for the size of the ee
                while (t <= tEndOfStance && t < tEnd) {
                    traj.addKnot(t, eePos);
                    t += dt;
                }
            } else {
                double tEndOfSwing = t + cpi.getTimeLeft();

                V3D finalEEPos = V3D(getCurrentOrUpcomingPlannedContactLocation(limb, tEndOfSwing));

                // when we first transition to a swing phase (at some time
                // sample t_i), this could be at the very beginning of it,
                // or it could have been a "little while ago" and so the
                // position of the swing foot at this moment in time needs
                // to account for this sampling-induced variation...
                bool firstTimeStepInSwingPhase = (cpi.getDuration() - cpi.getTimeLeft()) < dt;

                // plan motion for the entire swing phase here to avoid
                // computing redundant information...
                while (t <= tEndOfSwing) {
                    ContactPhaseInfo cpiSwing = cpm->getCPInformationFor(limb, t);

                    double factor = 1.0;
                    if (firstTimeStepInSwingPhase)
                        factor = (cpi.getDuration() - cpi.getTimeLeft()) / dt;
                    firstTimeStepInSwingPhase = false;

                    V3D oldEEPos = rawTraj.getKnotValue(rawTraj.getKnotCount() - 1);
                    // now, we have the remainder of the swing phase to go
                    // from the old step position to the final stepping
                    // location. Based on this we know how much we should be
                    // travelling over a time window dt...
                    double dTimeStep = 1.0;
                    // the -0.05 thing here means we want the swing foot to
                    // reach its target 0.05s before the swing phase ends
                    if (cpiSwing.getTimeLeft() - 0.001 > dt)
                        dTimeStep = dt / (cpiSwing.getTimeLeft() - 0.001) * factor;

                    V3D deltaStep = dTimeStep * (finalEEPos - oldEEPos);
                    V3D eePos = oldEEPos + deltaStep;
                    rawTraj.addKnot(t, V3D(eePos));
                    double groundHeight = ground.getHeight(eePos[0], eePos[2]);
                    // add ground height + ee size as offset...
                    double bFrameHeadingAngle = bFrameHeadingTrajectory.evaluate_catmull_rom(t);
                    eePos += getRotationQuaternion(bFrameHeadingAngle, V3D(0, 1, 0)) * lmp.generalSwingTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed());
                    eePos.y() = groundHeight + lmp.generalSwingTraj.evaluate_catmull_rom(cpiSwing.getPercentageOfTimeElapsed()).y() * lmp.swingFootHeight;

                    traj.addKnot(t, eePos);
                    t += dt;
                }
            }
        }

        return traj;
    }
};

/*
        The body frame motion represents the *average* linear trajectory and heading for a robot's trunk. 
        Roll, pitch and parasitic components of motion (e.g. transient fluctuations in yaw / translation 
        of COM arising from reactions to small perturbations or planned body sway, e.g. side to side motion during a 
        stride) are not captured by the motion of the body frame.
    */
class bFrameReferenceMotionPlan {
private:
    //0-2: xyz coords of position, in world coords
    //3: heading
    typedef Eigen::Matrix<double, 4, 1> bFrameState;

    void generate(const bFrameState& startingbFrameState, FootstepPlan fsp) {
        double headingAngle = startingbFrameState[3];
        P3D pos(startingbFrameState[0], startingbFrameState[1], startingbFrameState[2]);
        double vForward = std::clamp(targetForwardSpeed, 0.0, maxSpeed);
        double vSideways = targetSidewaysSpeed;
        double turningSpeed = targetTurngingSpeed;

        bFramePosTrajectory.clear();
        bFrameHeadingTrajectory.clear();
        bFrameVels.clear();

        double t = tStart;

        // generate trajectories that capture the motion of the robot's body frame...
        while (t < tEnd) {
            Quaternion heading = getRotationQuaternion(headingAngle, V3D(0, 1, 0));

            bFramePosTrajectory.addKnot(t, V3D(pos));
            bFrameHeadingTrajectory.addKnot(t, headingAngle);

            bFrameVels.addKnot(t, V3D(vForward, vSideways, turningSpeed));

            vForward = std::clamp(targetForwardSpeed, 0.0, maxSpeed);
            vSideways = targetSidewaysSpeed;
            turningSpeed = targetTurngingSpeed;

            // TODO: Trajectory Planning
            //
            // update "pos" and "headingAngle".
            // compute nextstep's "pos" and "headingAngle" by integrating "vForward", "vSideways" and "turningSpeed".
            //
            // Hints:
            // - quaternion heading = Roty(headingAngle). (we use y-up world frame coordinate)
            // - you can get the robot's forward direction vector from robot->forward
            // - you can get the robot's sideways direction vector by RBGlobals::worldUp.cross(robot->forward)

            // get rotation matrix around y axis with angle 'headingAngle'
            Matrix rot = Eigen::AngleAxisd(headingAngle, Eigen::Vector3d::UnitY()).toRotationMatrix();

            //calculate new position and angle
            pos.y = targetbFrameHeight + ground.getHeight(pos.x, pos.z);
            pos = pos + dt * (rot * V3D(0, 0, 1) * vForward + rot * RBGlobals::worldUp.cross(V3D(0, 0, 1)) * vSideways);
            headingAngle = headingAngle + dt * turningSpeed;

            t += dt;
        }


        LimbMotionProperties pelvisLmProps = LimbMotionProperties(robot->getLimbByName("pelvis"));
        if (tStart > 0.001) {
            Trajectory3D displacement = fsp.generateNonFootTrajectory(robot->getLimbByName("pelvis"), pelvisLmProps, tStart, tEnd, dt, bFramePosTrajectory, bFrameHeadingTrajectory);

            // Add the displacement trajectory to the bFrame trajectory per knot
            for (int i = 0; i < displacement.getKnotCount(); i++) {
                bFramePosTrajectory.setKnotValue(i, displacement.getKnotValue(i));
            }

        }
    }

public:
    //keep track of start, end and sampling rate for the motion trajectory
    double tStart = 0.0;
    double tEnd = 1.0;
    double dt = 1 / 30.0;

    //high level parameters that govern the resulting motion
    double targetForwardSpeed = 0;
    double targetSidewaysSpeed = 0;
    double targetTurngingSpeed = 0;
    double targetbFrameHeight = 0;

    //and the robot we apply this to
    std::shared_ptr<LeggedRobot> robot = nullptr;
    crl::gui::SizeableGroundModel ground = crl::gui::SizeableGroundModel(10);

    bFrameReferenceMotionPlan(const std::shared_ptr<LeggedRobot>& robot) {
        this->robot = robot;
    }

    // store reference linear motion for the robot's body
    Trajectory3D bFramePosTrajectory;
    //as well as a heading trajectory
    Trajectory1D bFrameHeadingTrajectory;
    //and the profile of turning, forward and sideways velocities that were used
    Trajectory3D bFrameVels;

    bFrameState getBFrameStateFromRBState(const RBState& rbState) {
        bFrameState s;
        BodyFrame bFrame(rbState.pos, rbState.orientation);

        s[0] = bFrame.p[0];
        s[1] = bFrame.p[1];
        s[2] = bFrame.p[2];
        s[3] = bFrame.h;
        return s;
    }

    bFrameState getBFrameStateAt(double t) {
        bFrameState s;
        P3D p = P3D() + bFramePosTrajectory.evaluate_catmull_rom(t);
        double h = bFrameHeadingTrajectory.evaluate_catmull_rom(t);
        V3D vels = bFrameVels.evaluate_catmull_rom(t);
        s[0] = p[0];
        s[1] = p[1];
        s[2] = p[2];
        s[3] = h;
        return s;
    }

    // compute the body frame position and heading based on the
    // position/orientation of the robot's trunk
    bFrameState getInitialConditionsFromCurrentTrunkState() {
        bFrameState initialBFrameState = getBFrameStateFromRBState(robot->getTrunk()->getState());
        return initialBFrameState;
    }

    bool hasGeneratedMotionTrajectory() {
        return bFramePosTrajectory.getKnotCount() > 0;
    }

    //generate the bFrame reference trajectory starting from the current state of the robot's trunk...
    void generateTrajectory(FootstepPlan fsp) {
        generate(getInitialConditionsFromCurrentTrunkState(), fsp);
    }

    void populateFootstepPlan(FootstepPlan& fsp, const LimbMotionProperties& lmProps, ContactPlanManager* cpm, double groundHeight = 0) {
        fsp.cpm = cpm;
        double tTiny = 0.0001;
        for (uint i = 0; i < robot->getLimbCount(); i++) {
            const auto& limb = robot->getLimb(i);
            fsp.footSteps[limb].clear();
            ContactPhaseInfo cpi = cpm->getCPInformationFor(limb, tStart);
            double t = tStart;
            if (cpi.isStance()) {
                P3D pos = limb->getEEWorldPos();
                pos.y = groundHeight + limb->ee->radius * lmProps.contactSafetyFactor;  // account for the size of the ee
                fsp.footSteps[limb].push_back(PlannedLimbContact(tStart, tStart + cpi.getTimeLeft(), pos, true));
                t = tStart + cpi.getTimeLeft() + tTiny;
            }

            while (t < tEnd) {
                ContactPhaseInfo cpiSwing = cpm->getCPInformationFor(limb, t);
                //when we get here, at time t, the foot is in swing, so find the start of the next contact phase
                if (cpiSwing.isSwing() == false) {
                    Logger::consolePrint(
                        "ERROR, ERROR, at this point the limb should be in "
                        "swing, but isn't");
                    assert(cpiSwing.isStance());
                }

                t += cpiSwing.getTimeLeft();
                ContactPhaseInfo cpiStance = cpm->getCPInformationFor(limb, t + tTiny);
                if (cpiStance.isStance() == false) {
                    Logger::consolePrint(
                        "ERROR, ERROR, at this point the limb should be in "
                        "stance, but isn't");
                    assert(cpiStance.isSwing());
                }

                double tStart = t;
                double tEnd = tStart + cpiStance.getDuration();
                t = tEnd + tTiny;

                //this is the moment in time where we'd like the limb to be right under its hip
                limb->normalizedSpeed = 1.0;
                if (targetForwardSpeed_shared != NULL){
                    limb->normalizedSpeed = std::clamp(*targetForwardSpeed_shared, 0.0, maxSpeed) / maxSpeed; // We should also allow negative speeds.
                }
                double speed = limb->normalizedSpeed;
                double tMidStance = tStart + cpiStance.getDuration() * (lmProps.ffStancePhaseForDefaultStepLength - 0.1 * speed);
                //so, compute the location of the body frame at that particular moment in time...
                double bFrameHeadingAngle = bFrameHeadingTrajectory.evaluate_catmull_rom(tMidStance);
                P3D bFramePos = P3D() + bFramePosTrajectory.evaluate_catmull_rom(tMidStance);

                V3D defaultEEOffset = limb->defaultEEOffset;
                defaultEEOffset[0] *= lmProps.stepWidthOffsetX;
                defaultEEOffset[2] *= lmProps.stepWidthOffsetZ;

                P3D pos = bFramePos + getRotationQuaternion(bFrameHeadingAngle, V3D(0, 1, 0)) * defaultEEOffset;

                pos.y = limb->ee->radius * lmProps.contactSafetyFactor;  // account for the size of the ee

                fsp.footSteps[limb].push_back(PlannedLimbContact(tStart, tEnd, pos, false));
            }
        }
    }
};

}  // namespace crl::loco
