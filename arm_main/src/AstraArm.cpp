#include <Arduino.h>
#include <AstraArm.h>
#include <FABRIK2D.h>


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
AstraArm::AstraArm(int segments[3], int ratios[3], float angles[3], float cur_pos[2]) {

    for(int i = 0; i < 3; i++){
        this->segments[i] = segments[i];
        this->ratios[i] = ratios[i];
        this->angles[i] = angles[i];
    }

    this->fabrik = Fabrik2D(4,segments,10.0F);

    this->cur_pos[0] = cur_pos[0];
    this->cur_pos[1] = cur_pos[1];

};

void AstraArm::IK_Execute(){
    //Execute the inverse kinematics for the arm (update arm speeds,angles,pos,etc..)
    if(this->ik_obj.active){

        this->fabrik.setJoints(this->angles, this->segments);   //Forward kinematics, get end effector position
        this->cur_pos[0] = this->fabrik.getX(3);                //Update current end effector position
        this->cur_pos[1] = this->fabrik.getY(3);                //

        //update joint angle speeds based on the IK objective


        if(updateJointSpeeds() == 1)
            this->ik_obj.active = false;//All joints are at their target angles, so the IK objective is complete
    }
    
};

int AstraArm::IK_Plan(float rel_target_x, float rel_target_y){
    this->ik_obj.active = true;

    //perform forward Kinematics
    this->fabrik.setJoints(this->angles, this->segments);
    
    
    this->cur_pos[0] = this->fabrik.getX(3);
    this->cur_pos[1] = this->fabrik.getY(3);

    this->ik_obj.target_pos[0] = cur_pos[0] + rel_target_x;
    this->ik_obj.target_pos[1] = cur_pos[1] + rel_target_y;

    this->ik_output = this->fabrik.solve(this->ik_obj.target_pos[0], this->ik_obj.target_pos[1], this->segments);
        if(this->ik_output == 0)
            return 0;//exit with 0 if the IK solution failed

    this->ik_obj.target_angles[0] = this->fabrik.getAngle(0);
    this->ik_obj.target_angles[1] = this->fabrik.getAngle(1);
    this->ik_obj.target_angles[2] = this->fabrik.getAngle(2);

    return 1;

    //may need to calculate & set joint motor speeds here?
    //UPDATE: This might not need to happen, since the motor speeds will be updated right after run with IK_Execute()
    

};

int AstraArm::updateJointSpeeds()//Returns 1 if all joints are at their target angles, 0 otherwise
{
    int goals_reached = 0;
    float axis_3_duty = 0.0;//duty for axis_3 (constant)
    float target_sec = 0.0;//time to reach target in seconds

    for(int i = 0; i < 3; i++)
    {
        //Left to be completed
        //set the speed of axis 0
        //determine how long axis 0 will take to reach its target
        //set speeds of the other motors to take just as long to reach their targets


        if((this->angles[i] > (this->ik_obj.target_angles[i]-2.0)) && (this->angles[i] < (this->ik_obj.target_angles[i]+2.0)))//if the joint is within 2 degrees of the target angle
        {
            this->ik_obj.motor_speeds[i] = 0;//stop the motor once it reaches within 2 degrees of the target angle
            goals_reached++;
            continue;//go to the next joint
        }
        
        if(this->ik_obj.motor_speeds[i]<0)//CCW
        {
            if(this->angles[i] > this->ik_obj.target_angles[i])//if the joint is past the target angle
            {
                this->ik_obj.motor_speeds[i] = 0;//stop the motor
                //this->angles[i] = this->ik_obj.target_angles[i];//set the joint angle to the target angle
                //goals_reached++;
                continue;//go to the next joint
            }
        }
        else//CW
        {
            if(this->angles[i] < this->ik_obj.target_angles[i])//if the joint is past the target angle
            {
                this->ik_obj.motor_speeds[i] = 0;//stop the motor
                //this->angles[i] = this->ik_obj.target_angles[i];//set the joint angle to the target angle
                //goals_reached++;
                continue;//go to the next joint
            }
        }

    }

    if(goals_reached == 3)//if all joints stopped, final goal reached (or overreached)
    {
        return 1;
    }else{
        return 0;
    }
}

AstraWrist::AstraWrist(){}; //default constructor
AstraWrist::AstraWrist(int max_tilt, float step_size){
    this->cur_tilt = 0;
    this->max_tilt = max_tilt;
    this->step_size = step_size;
};



