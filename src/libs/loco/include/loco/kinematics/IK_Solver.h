#pragma once

#include <loco/robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <loco/robot/Robot.h>

namespace crl::loco {

struct IK_EndEffectorTargets {
    std::shared_ptr<RB> rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

/* An enum class to specify the IK update rule */
enum class IK_UpdateRule {
    GAUSS_NEWTON,        // q_{k+1} = q_k + (J^T * J)^{-1} J^T * (pee_{target} - FK(q_k))
    LEVENBERG_MARQUARDT  // q_{k+1} = q_k + (J^T * J + lambda * I)^{-1} J^T * (pee_{target} - FK(q_k))
};

/* An enum class to specify the method we use to enforce joint limits. */
enum class IK_JointConstraintMethod { NONE, CLAMP, PROJECT };

class IK_Solver {
public:
    /**
     * @brief Construct a new IK solver object
     * 
     * @param robot the robot to solve IK for
     * @param constraintMethod which method to use to enforce joint limits
     * @param updateRule which update rule to use
     * @param alpha step size of the IK solver
     * @param lambda damping factor for the Levenberg-Marquardt update rule
     */
    IK_Solver(const std::shared_ptr<Robot> &robot, IK_UpdateRule updateRule = IK_UpdateRule::LEVENBERG_MARQUARDT,
              IK_JointConstraintMethod constraintMethod = IK_JointConstraintMethod::CLAMP, double alpha = 1.0, double lambda = 0.0001)
        : robot(robot), constraintMethod(constraintMethod), updateRule(updateRule), alpha(alpha), lambda(lambda) {}

    ~IK_Solver(void) {}

    /**
     * add IK end effector target to solver. Specify the end effector point p, which 
     * is specified in the local coordinates of rb and its target expressed in world frame.
     */
    void addEndEffectorTarget(const std::shared_ptr<RB> &rb, P3D p, P3D target) {
        endEffectorTargets.push_back(IK_EndEffectorTargets());
        endEffectorTargets.back().rb = rb;
        endEffectorTargets.back().p = p;
        endEffectorTargets.back().target = target;
    }

    void solve(int nSteps = 10) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);

        for (uint i = 0; i < nSteps; i++) {
            dVector q;
            gcrr.getQ(q);

            // get current generalized coordinates of the robots

            // TODO: Inverse Kinematics
            //
            // update generalized coordinates of the robot by solving IK.

            // remember, we don't update base pose since we assume it's already at
            // the target position and orientation

            // TODO: here, compute deltaq using Gauss-Newton.
            // end effector targets are stored in endEffectorTargets vector.
            //
            // Hint:
            // - use gcrr.estimate_linear_jacobian(p, rb, dpdq) function for Jacobian matrix.
            // - don't forget we use only last q.size() - 6 columns (use block(0,6,3,q.size() - 6) function)
            // - when you compute inverse of the matrix, use ldlt().solve() instead of inverse() function. this is numerically more stable.
            //   see https://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html

            // TODO: your implementation should be here.
            Matrix Jq, Jq_block;
            Matrix I = Matrix::Identity(q.size() - 6, q.size() - 6);
            for (size_t j = 0; j < endEffectorTargets.size(); j++) {
                // J(q)
                gcrr.estimate_linear_jacobian(endEffectorTargets[j].p, endEffectorTargets[j].rb, Jq);

                // FK(q)
                P3D FKq = gcrr.getWorldCoordinates(endEffectorTargets[j].p, endEffectorTargets[j].rb);

                // pee_target - FK(q)
                V3D difference = V3D(endEffectorTargets[j].target - FKq);

                // get relevant submatrix of dpdq
                Jq_block = Jq.block(0, 6, 3, q.size() - 6);

                // Update q (TODO: remove control flow once we settle on a final method to improve performance)
                if (updateRule == IK_UpdateRule::GAUSS_NEWTON) {
                    q.tail(q.size() - 6) += alpha * (Jq_block.transpose() * Jq_block).ldlt().solve(Jq_block.transpose() * difference).eval();
                } else if (updateRule == IK_UpdateRule::LEVENBERG_MARQUARDT) {
                    q.tail(q.size() - 6) += alpha * (Jq_block.transpose() * Jq_block + lambda * I).ldlt().solve(Jq_block.transpose() * difference).eval();
                } else {
                    throw std::invalid_argument("UpdateRule not implemented");
                }

                // Enforce joint angle constraints
                if (constraintMethod == IK_JointConstraintMethod::NONE) {
                    // do nothing
                    continue;
                } else if (constraintMethod == IK_JointConstraintMethod::CLAMP) {
                    for (int k = 0; k < q.size() - 6; k++) {
                        double minJointAngle = robot->getJoint(k)->minAngle;
                        double maxJointAngle = robot->getJoint(k)->maxAngle;
                        q[k + 6] = std::clamp(q[k + 6], minJointAngle, maxJointAngle);
                    }
                } else {
                    throw std::invalid_argument("JointConstraintMethod not implemented");
                }
            }

            // now update gcrr with q
            gcrr.setQ(q);
        }

        gcrr.syncRobotStateWithGeneralizedCoordinates();

        // clear end effector targets
        // we will add targets in the next step again.
        endEffectorTargets.clear();
    }

private:
    std::shared_ptr<Robot> robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
    IK_JointConstraintMethod constraintMethod;
    IK_UpdateRule updateRule;
    double alpha;
    double lambda;
};

}  // namespace crl::loco