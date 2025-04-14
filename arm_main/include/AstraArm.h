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
    inline void sendDuty(float duty0, float duty1, float duty2, float duty3) {
        MOTORSERIAL.printf("ctrl,%f,%f,%f,%f\n", duty0, duty1, duty2, duty3);
        lastDutyCycles[0] = duty0;
        lastDutyCycles[1] = duty1;
        lastDutyCycles[2] = duty2;
        lastDutyCycles[3] = duty3;
    }

   public:
    AstraArm(ArmJoint* setJoints[]);
    void setTargetAngles(float angle0, float angle1, float angle2, float angle3);
    void updateIKMotion();  // Functions same as updateForAcceleration()
    void runDuty(float dutyCycles[4]);

    inline void setTTG(int ttgMs) {
        for (int i = 0; i < 4; i++) {
            joints[i]->setTTG(ttgMs);
        }
    }
    inline void stop() {
        float dutyCycles[4] = {0};
        runDuty(dutyCycles);
    }
};
