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
        Arm();
        //u16 setTool(Mat4 tool);
        //Mat4 getTool();
    protected:
        POSE controls;
    private:
        
};

class IdealArm : public Arm {
    public:
        IdealArm();
        void setJointArray(Joint* jointArray);
        ControlMode update(ARM_CARTESIAN_VARIABLES controls);
        ControlMode setMovementMode(ControlMode mode);
        ControlMode getMovementMode();
    private:
        Joint* J;
        JointToMotor jointToMotor;
        float qJ[AXIS_COUNT];
        u16 qM[AXIS_COUNT];
        ControlMode controlMode;
};

class RealArm : public Arm {
    public:
        RealArm();
        void setHDArray(Mat4* dhArray);
        void setTransducerArray(Resolver* rArray);
        ARM_CARTESIAN_VARIABLES update();
    private:
        Mat4* H;
        Resolver* T;
};



#endif