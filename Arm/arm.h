#ifndef _ARM_H
#define _ARM_H

#include "mat.h"
#include "jointToMotor.h"
#include "joint.h"
#include "canDevice.h"

#define REACH_VEL 0.1f

enum ControlMode {
    NO_CONTROL_MODE,
    JOINT,
    CARTESIAN,
    TOOL,
};

class Arm{
    public:
        Arm(JointToMotor* jointToMotor);
        //u16 setTool(Mat4 tool);
        //Mat4 getTool();
    protected:
        POSE _controls;
        JointToMotor *jointToMotor;
    private:
        
};

class IdealArm : public Arm {
    public:
        IdealArm(JointToMotor* jointToMotor);
        void setJointArray(Joint* jointArray);
        void setMotorArray(Motor* motorArray);
        ControlMode update(POSE controls);
        ControlMode setMovementMode(ControlMode mode);
        ControlMode getMovementMode();
        POSE getPose();
        void updateState(u16 requestedMode, u16 controlMode, u16 currentLimits);
    private:
        ARM_JOINTS_VARIABLES inverseKin(POSE pose);
        Joint* J;
        Motor* M;
        u16 qM[AXIS_COUNT];
        ControlMode controlMode; 
};

class RealArm : public Arm {
    public:
        RealArm(JointToMotor* jointToMotor);
        void setDHArray(Mat4* dhArray);
        void setTransducerArray(Resolver* rArray);
        u16 getMotorPowerState();
        POSE update();   
        POSE getPose();
    private:
        ARM_CARTESIAN_VARIABLES forwardKin(ARM_JOINTS_VARIABLES pose);
        Mat4* H;
        Resolver* R;
        int motorCurrentMode[AXIS_COUNT];
};

void jointsControlTranslator(ARM_JOINTS_VARIABLES* armJoints, JOINT_CONTROL_TETRA* jointsControls);
void controlJointsTranslator(ARM_JOINTS_VARIABLES* armJoints, JOINT_CONTROL_TETRA* jointsControls);

#endif