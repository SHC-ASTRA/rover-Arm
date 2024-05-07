#include <Arduino.h>


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
        float segments[3]; //Length of each arm segment (units: mm)
        int ratios[3]; //Joint gear ratios (multiplier)
        float angles[3]; //Current joint angles (units: degrees)
        float cur_pos[2]; //Current end effector position (units: mm)
        Objective ik_obj; //IK Objective for the arm

        AstraArm(); //default constructor  
        AstraArm(float segments[3], int ratios[3], float angles[3], float cur_pos[2]); //constructor
};


class AstraWrist{
    public: 
        int cur_tilt; //Current tilt angle of the wrist (units: degrees)
        int max_tilt; //Current max tilt angle of the wrist (in both directions) (units: degrees)
        float step_size; //Step size for tilting/revolving the wrist (units: degrees per iteration) 

        AstraWrist(); //default constructor
        AstraWrist(int max_tilt, float step_size); //constructor
};