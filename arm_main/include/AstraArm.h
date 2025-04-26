/**
 * @file AstraArm.h
 * @author your name (you@domain.com)
 * @brief
 *
 */
#pragma once

#include <Arduino.h>
#include "ArmJoint.h"

#define MOTORSERIAL Serial1


class AstraArm {
   private:
    ArmJoint* joints[4];
    bool isIKMode;
    float lastDutyCycles[4];
    float lastVelocities[4];
    int timeToGoal;

    inline void sendDuty(float duty0, float duty1, float duty2, float duty3) {
        MOTORSERIAL.printf("ctrl,%f,%f,%f,%f\n", duty0, duty1, duty2, duty3);
        lastDutyCycles[0] = duty0;
        lastDutyCycles[1] = duty1;
        lastDutyCycles[2] = duty2;
        lastDutyCycles[3] = duty3;
    }
    inline void sendVelocity(float vel0, float vel1, float vel2, float vel3) {
        MOTORSERIAL.printf("sendvelocity,%f,%f,%f,%f\n", vel0, vel1, vel2, vel3);
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
