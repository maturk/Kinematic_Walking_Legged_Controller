#include <locomotion/LeggedRobot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>

#include <fstream>
#include <iostream>

#include "TestResult.h"

// I added these
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>
#include <utils/utils.h>
#include <robot/RobotState.h>
#include <utils/mathUtils.h>
#include <utils/mathDefs.h>

using namespace tests;

TestResult test_4_AnalyticJacobian() {
    crl::LeggedRobot robot(CRL_DATA_FOLDER "/robots/cora/cora_v4.rbs", nullptr,
                           false);
    robot.addLimb("fl", robot.getRBByName("tibia_0"));
    robot.addLimb("hl", robot.getRBByName("tibia_1"));
    robot.addLimb("fr", robot.getRBByName("tibia_2"));
    robot.addLimb("hr", robot.getRBByName("tibia_3"));

    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);

    crl::Matrix dpdq_analytic;
    crl::Matrix dpdq_estimated;

    TestResult res;

    for (uint i = 0; i < robot.limbs.size(); i++) {
        gcrr.compute_dpdq(robot.limbs[i]->ee->endEffectorOffset,
                          robot.limbs[i]->eeRB, dpdq_analytic);
        gcrr.estimate_linear_jacobian(robot.limbs[i]->ee->endEffectorOffset,
                                      robot.limbs[i]->eeRB, dpdq_estimated);

        // this shouldn't be all zero
        if (dpdq_analytic.isZero(0)) {
            res.passed = false;
            res.error = "\nAnalytic Jacobian is not implemented.\n";
        }

        // compare analytic jacobian with estimated one
        for (uint r = 0; r < dpdq_analytic.rows(); r++) {
            for (uint c = 0; c < dpdq_analytic.cols(); c++) {
                res += SAME(dpdq_analytic(r, c), dpdq_estimated(r, c), 1e-4);
            }
        }
    }

    return res;
}

TestResult test_1() {
    crl::LeggedRobot robot(CRL_DATA_FOLDER "/robots/cora/cora_v4.rbs", nullptr,
                           false);
    robot.addLimb("fl", robot.getRBByName("tibia_0"));
    robot.addLimb("hl", robot.getRBByName("tibia_1"));
    robot.addLimb("fr", robot.getRBByName("tibia_2"));
    robot.addLimb("hr", robot.getRBByName("tibia_3"));

    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);

    crl::Matrix dpdq_analytic;
    crl::Matrix dpdq_estimated;

    TestResult res;

    crl::P3D testPoint = crl::P3D(0.5,0.5,0.5);
    crl::P3D test ;
    std::cout <<"Parent of joint 19 is \n" <<gcrr.getParentQIdxOf(19)<<"\n\n";
      
    //std::cout <<"angle of 6 is " <<gcrr.getJointForQIdx(1)->getCurrentJointAngle()<<"\n\n";

    // std::cout <<"my first Function \n\n " <<gcrr.getCoordsInParentQIdxFrameAfterRotation(10,testPoint).y<<"\n\n";
    // //test = gcrr.getCoordsInParentQIdxFrameAfterRotation(4,testPoint);
    // std::cout<<" root index is :"<<robot.root->cJoints[1]->jIndex <<"\n\n\n";
    // std::cout<<" World coordinates is :" << robot.root->cJoints[3]->getWorldPosition()[3] <<"\n\n\n";
    // //::GeneralizedCoordinatesRobotRepresentation::getCoordsInParentQIdxFrameAfterRotation(10,testPoint);
    // //std::cout<<"TEST \n\n" << test.y << "\n\n\n";

    

    return res;
}

int main(int argc, char *argv[]) {
    // 4
    TEST(test_4_AnalyticJacobian);
    TEST(test_1);
    return (allTestsOk ? 0 : 1);
}
