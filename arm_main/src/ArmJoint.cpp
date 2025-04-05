/**
 * @file ArmJoint.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief 
 *
 */

#include "ArmJoint.h"


ArmJoint::ArmJoint(AS5047P* setEncoder, float setZeroAngle, int setGearRatio, bool setInverted) {
    encoder = setEncoder;
    zeroAngle = setZeroAngle;
    gearRatio = setGearRatio;
    inverted = setInverted;

    targetAngle = 0;
    lastEncoderAngle = 0;
    lastEncoderReadTime = 0;
    minAngle = -90;
    maxAngle = 90;
    timeToGoal = 5000;  // Default to 5 seconds
}

float ArmJoint::readAngle() {
    AS5047P_Types::ERROR_t errorInfo;
    lastEncoderAngle = encoder->readAngleDegree(true, &errorInfo);
    lastEncoderReadTime = millis();
    lastEffectiveAngle = lastEncoderAngle - zeroAngle;
    return lastEffectiveAngle;
}

float ArmJoint::updateIKMotion() {
    readAngle();
    float delta = targetAngle - lastEffectiveAngle;
    if (abs(delta) < PRECISION)
        return 0;  // Stop if +/- 1 degree

    float dutyCycle = delta / (maxAngle - minAngle);
    if (dutyCycle > 0.5) {
        dutyCycle = 0.5;
    } else if (dutyCycle < -0.5) {
        dutyCycle = -0.5;
    } else if (dutyCycle > 0 && dutyCycle < 0.1) {
        dutyCycle = 0.1;
    } else if (dutyCycle < 0 && dutyCycle > -0.1) {
        dutyCycle = -0.1;
    }
    if (inverted) {
        dutyCycle = -dutyCycle;
    }
    return dutyCycle;
}
