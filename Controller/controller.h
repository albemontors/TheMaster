#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm.h"

enum OpeMode {
    NO_MODE,
    NO_POW,
    IDLE,
    POWER_BEFORE_HOMED,
    POWER,
    MANUAL_BEFORE_HOMED,
    MANUAL,
    AUTOMATIC,
};

class ArmController{
    public:
        ArmController(u16p _MOTOR_DATA, u16p _MOTOR_SETTINGS, u16p _RESOLVER_DATA);
        OpeMode update();
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
        u16p MOTOR_DATA;
        u16p MOTOR_SETTINGS;
        u16p RESOLVER_DATA;
        OpeMode opeMode;
        OpeMode requestedMode;
        OpeMode forcedMode;
        void modeUpdate();
        CONTROLLER_ERROR error;
        bool noError();
        void setHome();
};

#endif