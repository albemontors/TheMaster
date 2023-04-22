#ifndef _CANDEVICE_H
#define _CANDEVICE_H

#include "definitions.h"

enum State {
    MOTOR_IDLE = 1,
    MOTOR_POW = 8,
};

class CanDevice{
    public:
        CanDevice();
        State getState();
    private:
        u16p canWrite;
        u16p canRead;
};

class Motor : public CanDevice{
    public:
        Motor();
        void setParams(M_PARAM params);
        u16 update(MOTOR_CONTROL_TETRA control);
    private:
        u16 maxSpeed;
        u16 maxAcc;
        u16 maxTorque;
};

class Resolver : public CanDevice {
    public:
        Resolver();
        void setParams(M_PARAM params);
        MOTOR_CONTROL_TETRA update();
        float* getQuotaPointer();
    private:
        float quota;
};

#endif