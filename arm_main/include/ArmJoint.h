/**
 * @file ArmJoint.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief 
 *
 */
#pragma once

#include <Arduino.h>
#include <AS5047P.h>

const float PRECISION = 1;


class ArmJoint {
//    private:
   public:
    float zeroAngle;
    float targetAngle;
    float lastEffectiveAngle;
    float lastEncoderAngle;
    long lastEncoderReadTime;
    float minAngle;
    float maxAngle;
    int timeToGoal;
    int gearRatio;
    bool inverted;
    AS5047P* encoder;

    public:
    ArmJoint(AS5047P* setEncoder, float setZeroAngle = 0, int setGearRatio = 1, bool setInverted = false);
    float readAngle();
    float updateIKMotion();

    inline void setTTG(int ttgMs) {
        timeToGoal = ttgMs;
    }
    inline void setTargetAngle(float angle) {
        targetAngle = angle;
        if (targetAngle < minAngle) {
            targetAngle = minAngle;
        } else if (targetAngle > maxAngle) {
            targetAngle = maxAngle;
        }
    }
};
