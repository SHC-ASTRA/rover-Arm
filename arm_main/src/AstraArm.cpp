#include <Arduino.h>
#include <AstraArm.h>


Objective::Objective(){}; //Default constructor
Objective::Objective(float target_pos[2], float target_angles[3], float motor_speeds[3]){
    
    
    for(int i = 0; i < 3; i++){
        this->target_angles[i] = target_angles[i];
        this->motor_speeds[i] = motor_speeds[i];
    }

    target_pos[0] = target_pos[0];
    target_pos[1] = target_pos[1];

    active = false;

};



AstraArm::AstraArm(){}; //Default constructor
AstraArm::AstraArm(float segments[3], int ratios[3], float angles[3], float cur_pos[2]) {

    for(int i = 0; i < 3; i++){
        this->segments[i] = segments[i];
        this->ratios[i] = ratios[i];
        this->angles[i] = angles[i];
    }

    this->cur_pos[0] = cur_pos[0];
    this->cur_pos[1] = cur_pos[1];

};

AstraWrist::AstraWrist(){}; //default constructor
AstraWrist::AstraWrist(int max_tilt, float step_size){
    this->cur_tilt = 0;
    this->max_tilt = max_tilt;
    this->step_size = step_size;
};



