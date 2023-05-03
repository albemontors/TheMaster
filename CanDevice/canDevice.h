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
    protected:
        State state;
        u16p canWrite;
        u16p canRead;        
        i16 maxSpeed;
        i16 maxTorque;        
};

class Motor : public CanDevice{
    public:
        Motor();
        void setParams(M_PARAM params);
        void update(MOTOR_CONTROL_TETRA control);
    private:

};

class Resolver : public CanDevice {
    public:
        Resolver();
        void setParams(M_PARAM params);
        MOTOR_STATE_TETRA update();
        float* getQuotaPointer();
    private:
        float quota;
};

#endif