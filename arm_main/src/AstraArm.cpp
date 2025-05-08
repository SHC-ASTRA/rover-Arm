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
    for (int i = 0; i < 4; i++) {
        lastDutyCycles[i] = 0;
    }
    timeToGoal = 5000;  // Default time to goal is 5 seconds
}

void AstraArm::setTargetAngles(float angle0, float angle1, float angle2, float angle3) {
    isIKMode = true;
    for (int i = 0; i < 4; i++) {
        joints[i]->timeToGoal = timeToGoal;
        joints[i]->goalTime = long(millis()) + timeToGoal;
    }
    joints[0]->setTargetAngle(angle0);
    joints[1]->setTargetAngle(angle1);
    joints[2]->setTargetAngle(angle2);
    joints[3]->setTargetAngle(angle3);
}

void AstraArm::updateIKMotion() {
    for (int i = 0; i < 4; i++) {
        joints[i]->readAngle();
    }

    if (!isIKMode) {
        // Check angle bounds
        bool wasLimitViolation = false;
        float newDutyCycles[4] = {lastDutyCycles[0], lastDutyCycles[1], lastDutyCycles[2], lastDutyCycles[3]};

        for (int i = 0; i < 4; i++) {
            if ((joints[i]->lastEffectiveAngle > joints[i]->maxAngle && lastDutyCycles[i] < 0)
             || (joints[i]->lastEffectiveAngle < joints[i]->minAngle && lastDutyCycles[i] > 0)) {
                wasLimitViolation = true;
                newDutyCycles[i] = 0;
            }
        }
        if (wasLimitViolation)
            sendDuty(newDutyCycles[0], newDutyCycles[1], newDutyCycles[2], newDutyCycles[3]);

        return;
    }

    float velocities[4] = {0};
    for (int i = 0; i < 4; i++) {
        velocities[i] = joints[i]->updateIKMotion();
    }
    sendVelocity(velocities[0], velocities[1], velocities[2], velocities[3]);
}

void AstraArm::runDuty(float dutyCycles[4]) {
    isIKMode = false;

    float newDutyCycles[4] = {0};

    // Check angle limits
    for (int i = 0; i < 4; i++) {
        if ((joints[i]->lastEffectiveAngle > joints[i]->maxAngle && dutyCycles[i] < 0)
         || (joints[i]->lastEffectiveAngle < joints[i]->minAngle && dutyCycles[i] > 0)) {
            newDutyCycles[i] = 0;
        } else {
            newDutyCycles[i] = dutyCycles[i];
        }
    }

    sendDuty(newDutyCycles[0], newDutyCycles[1], newDutyCycles[2], newDutyCycles[3]);
}
