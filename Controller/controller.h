#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm.h"

enum OpeMode {
    DEBUG = -2,
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
        void requestMode(OpeMode mode);
        void requestPower();
        void requestIdle();
        void requestManual(bool manual);
        void setControlMode(ControlMode controlMode);
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
        POSE i;
        POSE r;
        VECTOR6Df trackingError;
        VECTOR6Df maxAllowedTrackingError;
        DH_PARAM DH_param[AXIS_COUNT];
        J_PARAM J_param[AXIS_COUNT];
        M_PARAM M_param[AXIS_COUNT];
        u16p MOTOR_DATA;
        u16p MOTOR_SETTINGS;
        u16p RESOLVER_DATA;
        ControlMode controlMode;
        OpeMode opeMode;
        OpeMode requestedMode;
        OpeMode forcedMode;
        void modeUpdate();
        CONTROLLER_ERROR error;
        bool noError();
        void setHome();
};

#endif