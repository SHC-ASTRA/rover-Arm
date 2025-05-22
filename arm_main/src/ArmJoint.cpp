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
    goalTime = 0;
}

float ArmJoint::readAngle() {
    AS5047P_Types::ERROR_t errorInfo;
    lastEncoderAngle = clamp_angle(encoder->readAngleDegree(true, &errorInfo, true, true, true));
    if (errorInfo.noError()) {
        lastEncoderReadTime = millis();
        lastEffectiveAngle = clamp_angle(lastEncoderAngle - zeroAngle);
    }

    return lastEffectiveAngle;
}

double ArmJoint::pid(double pTargetAngle) {
    double error = clamp_angle(pTargetAngle - lastEffectiveAngle);
    if (abs(error) < PRECISION)
        return 0;  // Stop if +/- 1 degree
    
    error = (error / 360.0) * static_cast<double>(gearRatio);  // Convert to motor rotations from gearbox degrees

    integral += error * (dt / 1000.0);
    double derivative = (error - prevError) / (dt / 1000.0);
    prevError = error;

    return kP * error + kI * integral + kD * derivative;

    // const double degPerSec = error / (double(goalTime - long(millis())) / 1000.0);  // After gearbox
    // double motorRPM = (degPerSec * 60.0 / 360.0) * double(gearRatio);  // Before gearbox
}

float ArmJoint::updateIKMotion() {
    if (static_cast<long>(millis()) - lastEncoderReadTime > dt)
        return 0;  // Don't move if encoder data is too old

    double pidTargetAngle = targetAngle;  // Will come from S-curve

    float motorRPM = pid(targetAngle);
    if (motorRPM == 0) {
        return 0;
    }

    motorRPM = clamp_velocity(motorRPM);  // Keep non-zero velocity within MIN_SPEED and MAX_SPEED

    if (inverted) {
        motorRPM = -motorRPM;
    }
    return -1 * motorRPM;
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

float clamp_velocity(float velocity) {
    if (velocity > MAX_SPEED) {
        return MAX_SPEED;
    } else if (velocity < -MAX_SPEED) {
        return -MAX_SPEED;
    } else if (velocity > 0 && velocity < MIN_SPEED) {
        return MIN_SPEED;
    } else if (velocity < 0 && velocity > -MIN_SPEED) {
        return -MIN_SPEED;
    }
    return velocity;
}
