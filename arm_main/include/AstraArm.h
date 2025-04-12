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
    inline void sendDuty(float duty0, float duty1, float duty2, float duty3) {
        // MOTORSERIAL.printf("ctrl,%f,%f,%f,%f\n", duty0, duty1, duty2, duty3);
        MOTORSERIAL.printf("ctrl,%f,%f,%f\n", duty1, duty2, duty3);
    }

   public:
    AstraArm(ArmJoint* setJoints[]);
    void setTargetAngles(float angle0, float angle1, float angle2, float angle3);
    void updateIKMotion();  // Functions same as updateForAcceleration()

    inline void runDuty(float duty0, float duty1, float duty2, float duty3) {
        isIKMode = false;
        sendDuty(duty0, duty1, duty2, duty3);
    }
    inline void setTTG(int ttgMs) {
        for (int i = 0; i < 4; i++) {
            joints[i]->setTTG(ttgMs);
        }
    }
    inline void stop() {
        runDuty(0, 0, 0, 0);
    }
};
