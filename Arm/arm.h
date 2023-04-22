#ifndef _ARM_H
#define _ARM_H

#include "mat.h"
#include "jointToMotor.h"

class Arm{
    public:
        Arm();
        u16 setTool(Mat4 tool);
        Mat4 getTool();
    protected:
        POSE controls;
    private:
        
};

enum ControlMode {
    NO_POW,
    IDLE,
};

class IdealArm : public Arm {
    public:
        IdealArm();
        void setJointArray(Joint* jointArray);
        ControlMode update(VECTOR3Df controls);
        u16 setMovementMode(ControlMode mode);
        u16 getMovementMode(ControlMode mode);
    private:
        Joint* J;   //arm joints pointers
        JointToMotor jointToMotor;
        float qJ[AXIS_COUNT];
        u16 qM[AXIS_COUNT];
        ControlMode controlMode;
};

class RealArm : public Arm {
    public:
        RealArm();
        void setTransducerArray(Resolver* rArray);
        void setHDArray(Mat4* dhArray);
        VECTOR3Df update();
    private:
        Mat4* H;    //arm homogeneous transformations link-1 to link
        Resolver* T;
};

#endif