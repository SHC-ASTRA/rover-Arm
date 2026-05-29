/**
 * @file AstraArm.h
 * @author your name (you@domain.com)
 * @brief
 *
 */
#pragma once

#include <Arduino.h>
#include "ArmJoint.h"


class AstraArm {
   private:
    ArmJoint* joints[4];
    bool isIKMode;
    float lastDutyCycles[4];
    float lastVelocities[4];
    int timeToGoal;

    inline void sendDuty(float duty0, float duty1, float duty2, float duty3) {
#ifdef DEBUG
        Serial.printf("Sending duty cycles: %f, %f, %f, %f\n", duty0, duty1, duty2, duty3);
#endif
        joints[0]->motor->sendDuty(duty0);
        joints[1]->motor->sendDuty(duty1);
        joints[2]->motor->sendDuty(duty2);
        joints[3]->motor->sendDuty(duty3);
        lastDutyCycles[0] = duty0;
        lastDutyCycles[1] = duty1;
        lastDutyCycles[2] = duty2;
        lastDutyCycles[3] = duty3;
    }
    inline void sendVelocity(float vel0, float vel1, float vel2, float vel3) {
        vel0 = 0;
#ifdef DEBUG
        Serial.printf("Sending velocity commands: %f, %f, %f, %f\n", vel0, vel1, vel2, vel3);
#endif
        joints[0]->motor->sendSpeed(vel0);
        joints[1]->motor->sendSpeed(vel1);
        joints[2]->motor->sendSpeed(vel2);
        joints[3]->motor->sendSpeed(vel3);
        lastVelocities[0] = vel0;
        lastVelocities[1] = vel1;
        lastVelocities[2] = vel2;
        lastVelocities[3] = vel3;
    }

   public:
    AstraArm(ArmJoint* setJoints[]);
    void setTargetAngles(float angle0, float angle1, float angle2, float angle3);
    void updateIKMotion();  // Functions same as updateForAcceleration()
    void runDuty(float dutyCycles[4]);

    inline void setTTG(int ttgMs) {
        timeToGoal = ttgMs;
    }
    inline void stop() {
        float dutyCycles[4] = {0};
        runDuty(dutyCycles);
    }
};
