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
    targetVelocity = 0;
    setpointType = SETPOINT_ANGLE;
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

    // Calculate motor RPM required to achieve either absolute angle setpoint or velocity setpoint
    float motorRPM = 0;
    if (setpointType == SETPOINT_ANGLE) {
        double pidTargetAngle = targetAngle;  // Will come from S-curve

        motorRPM = pid(targetAngle);
    } else {
        if (targetVelocity == 0)
            return 0;

        // Perform bounds checking -- joint shall not exceed min/max angles within one second
        float projectedAngle = lastEffectiveAngle + targetVelocity;
        if (projectedAngle < minAngle) {
            targetVelocity = minAngle - lastEffectiveAngle;
        } else if (projectedAngle > maxAngle) {
            targetVelocity = maxAngle - lastEffectiveAngle;
        }

        // Convert deg/sec to motor RPM
        motorRPM = (targetVelocity * 60.0 / 360.0) * float(gearRatio);
    }
    if (motorRPM == 0)
        return 0;

    motorRPM = clamp_velocity(motorRPM);  // Bound motor RPM within MIN_SPEED and MAX_SPEED

    if (inverted)
        motorRPM *= -1;

    return -1 * motorRPM;  // Motors turn in the opposite direction of our joint angle convention
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
