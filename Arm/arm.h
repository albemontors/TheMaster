#ifndef _ARM_H
#define _ARM_H

#include "mat.h"
#include "jointToMotor.h"
#include "joint.h"
#include "canDevice.h"

enum ControlMode {
    NO_CONTROL_MODE,
    JOINT,
    CARTESIAN,
    TOOL,
};

class Arm{
    public:
        Arm(); // init function, leave it empity
        //u16 setTool(Mat4 tool);
        //Mat4 getTool();
    protected:
        POSE controls;
    private:
        
};

class IdealArm : public Arm {
    public:
        IdealArm(); // init function, leave it empity
        void setJointArray(Joint* jointArray); // gets the joint pointer and assigns it
        ControlMode update(ARM_CARTESIAN_VARIABLES controls); // i do this shit, leave it empity
        ControlMode setMovementMode(ControlMode mode); // set the control mode param, returns the new control mode (checks to be added later)
        ControlMode getMovementMode(); // get the control mode param
    private:
        Joint* J;   //arm joints pointers
        JointToMotor jointToMotor;
        float qJ[AXIS_COUNT];
        u16 qM[AXIS_COUNT];
        ControlMode controlMode;
};

class RealArm : public Arm {
    public:
        RealArm(); // init function, leave it empity
        void setTransducerArray(Resolver* rArray); // sets the T variable with the pointer
        void setHDArray(Mat4* dhArray); // sets the H variable with the pointer
        ARM_CARTESIAN_VARIABLES update(); // i do this shit, leave it empity
    private:
        Mat4* H;    //arm homogeneous transformations link-1 to link
        Resolver* T;
};



#endif