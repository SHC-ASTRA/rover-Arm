/**
 * @file ArmJoint.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief 
 *
 */
#pragma once

#include <Arduino.h>
#include <AS5047P.h>

enum SetpointType {
    SETPOINT_ANGLE,
    SETPOINT_VELOCITY
};


const float PRECISION = 1;

const float MAX_SPEED = 500;
const float MIN_SPEED = 100;

const float dt = 50;  // ms
const float kP = 10.0;
const float kI = 0.0;
const float kD = 0.0;

/**
 * @brief Clamps angle between -180 and +180 degrees
 * 
 * @param angle degrees
 * @return float degrees
 */
float clamp_angle(float angle);

/**
 * @brief enforces MAX_SPEED and MIN_SPEED motor RPM limits
 * 
 * @param velocity RPM
 * @return float clamped RPM
 */
float clamp_velocity(float velocity);


class ArmJoint {
//    private:
   public:
    float zeroAngle;            // Raw encoder angle reading that corresponds to 0 degrees
    float minAngle;
    float maxAngle;

    SetpointType setpointType;  // Angle (deg) or angular velocity (deg/s)
    float targetAngle;          // Degrees from zeroAngle
    float lastEffectiveAngle;   // Last encoder angle reading, adjusted with zeroAngle
    float lastEncoderAngle;     // Last raw encoder angle reading
    long lastEncoderReadTime;   // millis() value of last encoder read
    float targetVelocity;       // Degrees per second
    float lastREVVelocity;      // Last motor RPM reported directly by the Sparkmax
    float lastDegSVelocity;     // Last joint velocity in deg/s (converted from lastREVVelocity)
    long lastREVReadTime;       // millis() value of last Sparkmax velocity read

    float integral;
    float prevError;

    long timeToGoal;
    long goalTime;  // millis() value when the arm should be at its goal

    int gearRatio;
    bool inverted;
    AS5047P* encoder;

    double pid(double pTargetAngle);

   public:
    ArmJoint(AS5047P* setEncoder, float setZeroAngle = 0, float setMinAngle = -115, float setMaxAngle = 115, int setGearRatio = 1, bool setInverted = false);
    float readAngle();
    void readREVVelocity(int rpm);
    float updateIKMotion();

    inline void setTargetAngle(float angle) {
        targetAngle = angle;
        setpointType = SETPOINT_ANGLE;
        if (targetAngle < minAngle) {
            targetAngle = minAngle;
        } else if (targetAngle > maxAngle) {
            targetAngle = maxAngle;
        }
    }

    inline void setTargetVelocity(float velocity) {
        targetVelocity = velocity;
        setpointType = SETPOINT_VELOCITY;
    }

    inline bool checkDuty(float duty) {
        if (inverted)
            duty = -duty;
        if ((lastEffectiveAngle > maxAngle && duty < 0)
            || (lastEffectiveAngle < minAngle && duty > 0)) {
            return false;
        }
        return true;
    }
};
