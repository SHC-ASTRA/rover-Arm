/**
 * @file ArmJoint.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief 
 *
 */

#include "ArmJoint.h"


ArmJoint::ArmJoint(AS5047P* setEncoder, float setZeroAngle, float setMinAngle, float setMaxAngle, int setGearRatio, bool setInverted) {
    encoder = setEncoder;
    zeroAngle = setZeroAngle;
    gearRatio = setGearRatio;
    inverted = setInverted;

    targetAngle = 0;
    lastEncoderAngle = 0;
    lastEncoderReadTime = 0;
    minAngle = setMinAngle;
    maxAngle = setMaxAngle;
    timeToGoal = 5000;  // Default to 5 seconds
}

float ArmJoint::readAngle() {
    AS5047P_Types::ERROR_t errorInfo;
    lastEncoderAngle = clamp_angle(encoder->readAngleDegree(true, &errorInfo));
    lastEncoderReadTime = millis();

    lastEffectiveAngle = clamp_angle(lastEncoderAngle - zeroAngle);

    return lastEffectiveAngle;
}

float ArmJoint::updateIKMotion() {
    float delta = targetAngle - lastEffectiveAngle;
    if (abs(delta) < PRECISION)
        return 0;  // Stop if +/- 1 degree

    float dutyCycle = SPEED_MULT * delta / (maxAngle - minAngle);
    if (dutyCycle > MAX_SPEED) {
        dutyCycle = MAX_SPEED;
    } else if (dutyCycle < -MAX_SPEED) {
        dutyCycle = -MAX_SPEED;
    } else if (dutyCycle > 0 && dutyCycle < MIN_SPEED) {
        dutyCycle = MIN_SPEED;
    } else if (dutyCycle < 0 && dutyCycle > -MIN_SPEED) {
        dutyCycle = -MIN_SPEED;
    }

    if (inverted) {
        dutyCycle = -dutyCycle;
    }
    return -1 * dutyCycle;
}


float clamp_angle(float angle) {
    angle = fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360;
    }
    if (angle > 180) {
        angle -= 360;
    }
    return angle;
}
