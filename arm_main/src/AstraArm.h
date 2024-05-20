#include <Arduino.h>
//#include <AstraWrist.h>
#include <FABRIK2D.h>
#include <LSS.h>

class Objective{
    public:
        float target_pos[2];
        float target_angles[3];
        float motor_speeds[3];
        bool active;

        Objective(); //default constructor
        Objective(float target_pos[2], float target_angles[3], float motor_speeds[3]); //constructor
};

class AstraWrist{
    public: 
        int cur_tilt; //Current tilt angle of the wrist (units: degrees)
        int max_tilt; //Current max tilt angle of the wrist (in both directions) (units: degrees)
        int rpm; //Step size for tilting/revolving the wrist (units: degrees per iteration) 

        AstraWrist(); //default constructor
        AstraWrist(int max_tilt, int rpm); //constructor
        //void move_wrist(bool revolve, bool invert, LSS &top_lss, LSS &bottom_lss);
};

class AstraArm{
    public:
        int segments[3]; //Length of each arm segment (units: mm)
        int ratios[3]; //Joint gear ratios (multiplier)
        float angles[3]; //Current joint angles (units: degrees)
        float cur_pos[2]; //Current end effector position (units: mm)


        //Constants for calculating speed for a given duty cycle
        double ax_1_k1 = 0.1545315; // Axis 1 (3x 5:1 + 40:1 Cycloidal) 
        double ax_1_k2 = 0.3264763; // deg/sec = k1 x (%DUTY) - k2

        double ax_2_k1 = 0.3260635; // Axis 2 (3x 5:1 + 20:1 Cycloidal) 
        double ax_2_k2 = 0.1686907; // deg/sec = k1 x (%DUTY) - k2

        double ax_3_k1 = 0.4893003; // Axis 3 (3x 5:1 + 12:1 Cycloidal) 
        double ax_3_k2 = 0.4609032; // deg/sec = k1 x (%DUTY) - k2

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








