#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm.h"

class ArmController{
    public:
        ArmController();
        ControlMode update();
        void requestMode();
        void requestPower();
        void requestIdle();
        void requestManual(bool manual);
        void requestAutomatic(bool automatic);
    private:
        Joint J[AXIS_COUNT];
        Motor M[AXIS_COUNT];
        Mat4 DH[AXIS_COUNT];
        Resolver R[AXIS_COUNT];
        JointToMotor jointToMotor;
        RealArm realArm;
        IdealArm idealArm;
        float k;
        ARM_CARTESIAN_VARIABLES iC;
        ARM_CARTESIAN_VARIABLES rC;
        VECTOR3Df trackingError;
        VECTOR3Df maxAllowedTrackingError;
        DH_PARAM DH_param[AXIS_COUNT];
        J_PARAM J_param[AXIS_COUNT];
        M_PARAM M_param[AXIS_COUNT];
        ControlMode controlMode;
        ControlMode requestedMode;
        ControlMode forcedMode;
        void modeUpdate();
        CONTROLLER_ERROR error;
        bool noError();
        void setHome();
};

#endif