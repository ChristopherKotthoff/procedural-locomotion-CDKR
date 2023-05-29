#pragma once

#include "loco/robot/GeneralizedCoordinatesRobotRepresentation.h"
#include "loco/robot/Robot.h"
#include "loco/robot/RobotState.h"

namespace crl::loco {

/**
 * This class represents a generic limb (i.e. leg or arm). Each limb has an
 * origin RB, a bunch of links connected by joints, and an end effector RB.
 */
struct RobotLimb {
    // index of limb
    int limbIndex = -1;
    // this is the name of the limb
    std::string name;
    // and all limbs have an end effector
    std::shared_ptr<RB> eeRB = nullptr;
    // and this is a list of all the limb's joints - for easy access...
    std::vector<std::shared_ptr<RBJoint>> jointList;

    // this is the vector from the CoM of the trunk to the end effector; it
    // corresponds to a default step offset. Expressed in the coordinate frame
    // of the trunk...
    V3D defaultEEOffset;
    // this is ptr to ee we use for locomotion related tasks
    RBEndEffector *ee = nullptr;

    V3D phase0; 
    V3D phase1;
    V3D phase2; 
    V3D phase3; 
    V3D phase4;
    V3D phase5;
    double normalizedSpeed = 1;

    double height0 = 0;
    double height1 = 4;
    double height2 = 0;
    double offset0 = 1.0; 
    double offset1 = 1.0;
    double offset2 = 0.7;

    double time0; 
    double time1;
    double time2; 
    double time3; 
    double time4;
    double time5; 

    double headBop = 0.005;
    double headSpeedScaler = 0.2;

    double yMaxForBase = 0.01; 
    double yMaxForScaler = 0.5 * 0.02;
    double zMaxForBase = 0.005; 
    double zMaxForScaler = 0.5 * 0.035;
    double zMaxBackBase = 0.0175; 
    double zMaxBackScaler = 0.5 * 0.035;
    double yMaxBackBase = 0.005; 
    double yMaxBackScaler = 0.5 * 0.01;
    double yMinMidBase = 0.0025; 
    double yMinMidScaler = 0.5 * 0.05;
    double xHandInBase = 0.005; 
    double xHandInScaler = 0.5 * 0.01;

    double pelvisBop = 0.05;
    double pelvisShift = 0;


    /**
     * constructor: looking for EE
     */
    RobotLimb(const std::string &name, const std::shared_ptr<RB> &eeRB, const std::shared_ptr<RB> &limbRoot) {
        this->name = name;
        this->eeRB = eeRB;
        this->ee = &eeRB->rbProps.endEffectorPoints[0];

        std::shared_ptr<RB> tmpRB = eeRB;
        while (tmpRB != limbRoot) {
            jointList.insert(jointList.begin(), tmpRB->pJoint);
            tmpRB = tmpRB->pJoint->parent;
        }

        P3D eePos = ee->endEffectorOffset;
        defaultEEOffset = limbRoot->getLocalCoordinates(V3D(limbRoot->getWorldCoordinates(P3D()), eeRB->getWorldCoordinates(eePos)));
        bool is_leg = this->name == "lLowerLeg" || this->name == "rLowerLeg" || this->name == "lToes" || this->name == "rToes";
        bool is_hand = this->name == "lHand" || this->name == "rHand";
        bool is_head = this->name == "head";
        bool is_pelvis = this->name == "pelvis";

        if (is_leg) {
            
        } else if (is_hand) {
            double yMaxFor = this->yMaxForBase + this->yMaxForScaler * this->normalizedSpeed;
            double zMaxFor = this->zMaxForBase + 0.5 + this->zMaxForScaler  * this->normalizedSpeed;
            double zMaxBack = this->zMaxBackBase + this->zMaxBackScaler * this->normalizedSpeed;
            double yMaxBack = this->yMaxBackBase + 0.5 + this->yMaxBackScaler * this->normalizedSpeed;
            double yMinMid = this->yMinMidBase+ this->yMinMidScaler * this->normalizedSpeed;
            double xHandIn = this->xHandInBase + this->xHandInBase * this->normalizedSpeed;
            if (this->name == "lHand") { // Bit ugly, but both hands need to face inwards.
                xHandIn = -xHandIn;
            }
            this->phase0 = V3D(0, yMinMid, (-zMaxBack + zMaxFor) / 2);
            this->phase1 = V3D(0, yMaxBack, -zMaxBack);
            this->phase2 = V3D(0, yMinMid, (-zMaxBack + zMaxFor) / 2);
            this->phase3 = V3D(xHandIn, yMaxFor, zMaxFor);
            this->phase4 = V3D(0, yMinMid, (-zMaxBack + zMaxFor) / 2); // Meet in the middle
            this->time0 = 0;
            this->time1 = 0.125;
            this->time2 = 0.375;
            this->time3 = 0.635;
            this->time4 = 0.875;
        } else if (is_head) {
            double headLeanForward = this->normalizedSpeed * this->headSpeedScaler;

            this->phase0 = V3D(0, 0, headLeanForward);
            this->phase1 = V3D(0, this->headBop, headLeanForward);
            this->phase2 = V3D(0, -this->headBop, headLeanForward);
            this->phase3 = V3D(0, this->headBop, headLeanForward);
            this->phase4 = V3D(0, -this->headBop, headLeanForward);
            this->phase5 = V3D(0, 0, headLeanForward);
            this->time0 = 0;
            this->time1 = 0.125;
            this->time2 = 0.375;
            this->time3 = 0.635;
            this->time4 = 0.875;
            this->time5 = 1.0;
        } else if (is_pelvis) {
            double pelvisBop = 0.05 ;
            double shift = 0.0;
            this->phase0 = V3D(0, -pelvisBop,0);
            this->phase1 = V3D(0, -2*pelvisBop, 0);
            this->phase2 = (0.375 + shift, V3D(0, 0,0));
            this->phase3 = V3D(0, -2*pelvisBop, 0);
            this->phase4 = V3D(0, 0, 0);
            this->phase5 = (0, -pelvisBop, 0);
        } else {
            assert(false && "LimbMotionProperties: unknown limb type");
        }
    }

    /**
     * this corresponds to the hip or shoulder joint...
     */
    std::shared_ptr<RBJoint> getJointToTrunk() {
        return jointList[0];
    }

    P3D getEEWorldPos() const {
        return eeRB->getWorldCoordinates(this->ee->endEffectorOffset);
    }

    V3D getEEWorldVel() const {
        return eeRB->getVelocityForPoint_local(this->ee->endEffectorOffset);
    }

    bool isContact() const {
        return this->ee->inContact;
    }
};

/**
 * Each legged robot has a set of limbs and a trunk.
 */
class LeggedRobot : public Robot {
private:
    // we will assume that the trunk is just one rigid body for now, and that it
    // is the root of the robot...
    std::shared_ptr<RB> trunk = nullptr;
    std::vector<std::shared_ptr<RobotLimb>> limbs;

    // it's useful to store standing pose as a nominal state
    // if it is not specified then just set initial state of robot (but it might
    // be dangerous...)
    RobotState standingState;

public:
    explicit LeggedRobot(const char *filePath, const char *statePath = nullptr);

    ~LeggedRobot() override = default;

    std::shared_ptr<RB> getTrunk();

    void addLimb(const std::string &name, const std::shared_ptr<RB> &eeRB);

    void addLimb(const std::string &name, const std::string &eeRBName);

    int getLimbCount() const;

    std::shared_ptr<RobotLimb> getLimb(uint i) const;

    /**
     * Search the limb corresponding to the queried name
     */
    std::shared_ptr<RobotLimb> getLimbByName(const std::string &name) const;
};

}  // namespace crl::loco
