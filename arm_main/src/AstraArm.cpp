/**
 * @file AstraArm.cpp
 * @author your name (you@domain.com)
 * @brief 
 *
 */

#include "AstraArm.h"


AstraArm::AstraArm(ArmJoint* setJoints[4]) {
    for (int i = 0; i < 4; i++) {
        joints[i] = setJoints[i];
    }
    isIKMode = false;
}

void AstraArm::setTargetAngles(float angle0, float angle1, float angle2, float angle3) {
    isIKMode = true;
    joints[0]->setTargetAngle(angle0);
    joints[1]->setTargetAngle(angle1);
    joints[2]->setTargetAngle(angle2);
    joints[3]->setTargetAngle(angle3);
}

void AstraArm::updateIKMotion() {
    for (int i = 0; i < 4; i++) {
        joints[i]->readAngle();
    }
    if (!isIKMode)
        return;
    float dutycycles[4] = {0};
    for (int i = 0; i < 4; i++) {
        dutycycles[i] = joints[i]->updateIKMotion();
    }
    sendDuty(dutycycles[0], dutycycles[1], dutycycles[2], dutycycles[3]);
}
