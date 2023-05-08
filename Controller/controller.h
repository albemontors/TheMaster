#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm.h"
#include <cstdint>

#define CURRENT_LIMIT 60

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

typedef struct {
    float LX;
    float LY;
    float RX;
    float RY; }
INPUT;

class ArmController{
    public:
        ArmController(u16* _MOTOR_DATA, u16* _MOTOR_SETTINGS, u16* _RESOLVER_DATA);
        OpeMode update(INPUT input);
        void requestMode(OpeMode mode);
        OpeMode getMode();
        void setInput(VECTOR6Df input);
        void requestManual(bool manual);
        void setControlMode(ControlMode controlMode);
        void setManJogSpeed(float _manJogSpeed);
        void requestAutomatic(bool automatic);
        void setHome();
        POSE getIdealPose();
        POSE getRealPose();
    private:
        Joint J[AXIS_COUNT];
        Motor M[AXIS_COUNT];
        Mat4 DH[AXIS_COUNT];
        Resolver R[AXIS_COUNT];
        JointToMotor jointToMotor;
        RealArm realArm;
        IdealArm idealArm;
        float k;
        float manJogSpeed;
        VECTOR6Df trackingError;
        VECTOR6Df maxAllowedTrackingError;
        VECTOR6Df input;
        DH_PARAM DH_param[AXIS_COUNT];
        J_PARAM J_param[AXIS_COUNT];
        M_PARAM M_param[AXIS_COUNT];
        u16* MOTOR_DATA;
        u16* MOTOR_SETTINGS;
        uint16_t* RESOLVER_DATA;
        OpeMode opeMode;
        OpeMode requestedMode;
        OpeMode forcedMode;
        void modeUpdate();
        CONTROLLER_ERROR error;
        bool noError();
        void sendMotorControlWords(OpeMode mode);
        ARM_CARTESIAN_VARIABLES cartLimit[2];
        bool inCartSpace(ARM_CARTESIAN_VARIABLES input);
};

void sendSettingsPdos();

#endif