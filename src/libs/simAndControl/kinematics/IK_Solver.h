#pragma once

#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/Robot.h>

namespace crl {

struct IK_EndEffectorTargets {
    RB *rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

class IK_Solver {
public:
    IK_Solver(Robot *robot) : robot(robot) {}

    ~IK_Solver(void) {}

    /**
     * add IK end effector target to solver. Specify the end effector point p, which 
     * is specified in the local coordinates of rb and its target expressed in world frame.
     */
    void addEndEffectorTarget(RB *rb, P3D p, P3D target) {
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

            // TODO: Ex.2-2 Inverse Kinematics
            //
            // update generalized coordinates of the robot by solving IK.

            // remember, we don't update base pose since we assume it's already at
            // the target position and orientation
            dVector deltaq(q.size() - 6);
            deltaq.setZero();

            // TODO: here, compute deltaq using GD, Newton, or Gauss-Newton.
            // end effector targets are stored in endEffectorTargets vector.
            //
            // Hint:
            // - use gcrr.estimate_linear_jacobian(p, rb, dpdq) function for Jacobian matrix.
            // - if you already implemented analytic Jacobian, you can use gcrr.compute_dpdq(const P3D &p, RB *rb, Matrix &dpdq)
            // - don't forget we use only last q.size() - 6 columns (use block(0,6,3,q.size() - 6) function)
            // - when you compute inverse of the matrix, use ldlt().solve() instead of inverse() function. this is numerically more stable.
            //   see https://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html

            // TODO: your implementation should be here.
            //double e = 0.5*(endEffectorPosition(*linkage, x) - *target).dot((endEffectorPosition(*linkage, x) - *target));
            
            //Gauss -Newton:
            //qi+1 = qi + inv(J.T*J) * J.T * (PeeTarget - FK(q));
            //initialize dp/dq
            // Matrix dpdq = Matrix::Zero(3,q.size());
            // gcrr.estimate_linear_jacobian(endEffectorTargets.back().p, endEffectorTargets.back().rb, dpdq);
            // Matrix J = Matrix::Zero(3,q.size()-6);
            // J = dpdq.block(0,6,3,q.size() - 6).eval();
            // P3D difference = (endEffectorTargets.back().target - gcrr.getWorldCoordinates(endEffectorTargets.back().p, endEffectorTargets.back().rb))
            // Matrix myIdentity = Matrix::Identity((int)q.size()-6,(int)q.size()-6);
            // Matrix myPseudoInverse = (J.transpose() * J).ldlt().solve(myIdentity)*J.transpose();
            // Matrix deltaqIntermediate = Matrix::Zero(3,q.size());  
            // deltaqIntermediate = (myPseudoInverse * V3D(difference));
            // deltaq = ( (myPseudoInverse * V3D(difference))).eval();
            
            Matrix dpdq(3,q.size());
            for(int i = 0; i < endEffectorTargets.size(); i++)
            {
                //Gauss-Newton:
                //qi+1 = qi + inv(J.T*J) * J.T * (PeeTarget - FK(q));
                P3D pLocal = endEffectorTargets[i].p;
                V3D PeeTarget = V3D(endEffectorTargets[i].target);
                RB *currentRB = endEffectorTargets[i].rb;
                V3D FK = V3D(gcrr.getWorldCoordinates(pLocal,currentRB));

                gcrr.estimate_linear_jacobian(pLocal, currentRB, dpdq);

                Matrix J = dpdq.block(0,6,3,q.size() - 6);
                deltaq += (J.transpose() *J).ldlt().solve(J.transpose() * (PeeTarget - FK));
            }

            q.tail(q.size() - 6) += deltaq;

            // now update gcrr with q
            gcrr.setQ(q);
            
        }
        
        gcrr.syncRobotStateWithGeneralizedCoordinates();

        // clear end effector targets
        // we will add targets in the next step again.
        endEffectorTargets.clear();
    }

private:
    Robot *robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
};

}  // namespace crl