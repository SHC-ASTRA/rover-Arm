#include <Arduino.h>
#include <LSS.h>
#include <FABRIK2D.h>

class Objective{
    public:
        float target_pos[2];
        float target_angles[3];
        float motor_speeds[3];
        bool active;

        Objective(); //default constructor
        Objective(float target_pos[2], float target_angles[3], float motor_speeds[3]); //constructor
};


class AstraArm{
    public:
        int segments[3]; //Length of each arm segment (units: mm)
        int ratios[3]; //Joint gear ratios (multiplier)
        float angles[3]; //Current joint angles (units: degrees)
        float cur_pos[2]; //Current end effector position (units: mm)

        int ik_output;
        Objective ik_obj; //IK Objective for the arm
        AstraWrist wrist; //Wrist object for the arm (NOT CURRENTLY IN USE, TO BE IMPLEMENTED FOR IK)
        Fabrik2D fabrik; //FABRIK2D object for the arm


        AstraArm(); //default constructor  
        AstraArm(int segments[3], int ratios[3], float angles[3], float cur_pos[2]); //constructor

        void IK_Execute(); //Execute the inverse kinematics for the arm (update arm speeds,angles,pos,etc..)
        int IK_Plan(float rel_target_x, float rel_target_y); //Plan the inverse kinematics for the arm (based on the set objective). This should run when new control commands are received.
        int updateJointSpeeds(); //Update the joint speeds such that they'll reach their target angles together
};


class AstraWrist{
    public: 
        int cur_tilt; //Current tilt angle of the wrist (units: degrees)
        int max_tilt; //Current max tilt angle of the wrist (in both directions) (units: degrees)
        float step_size; //Step size for tilting/revolving the wrist (units: degrees per iteration) 

        AstraWrist(); //default constructor
        AstraWrist(int max_tilt, float step_size); //constructor
};


void move_wrist(AstraWrist &wrist, bool revolve, bool invert, LSS &top_lss, LSS &bottom_lss){
    float angle = wrist.step_size * 10;//convert to 0.1 degree increments
    if(invert)
    {
        angle *= -1;//tilt left / revolve ccw
    }

    if(revolve){//if true, revolve (opposite absolute directions)
        top_lss.moveRelative(angle*3);
        bottom_lss.moveRelative(angle*3);
    }else{
        if(wrist.cur_tilt+angle > wrist.max_tilt || wrist.cur_tilt+angle < -wrist.max_tilt){
            return;//max angle would be exceeded, don't move the motors
        }
        top_lss.moveRelative(angle);
        bottom_lss.moveRelative(angle*-1);
        wrist.cur_tilt += angle;
    }
}


