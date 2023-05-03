#ifndef _JOINT_H
#define _JOINT_H

#include "definitions.h"

class Joint{
    public:
        Joint();
        void setParams(J_PARAM params);
        JOINT_STATE update(JOINT_CONTROL_TETRA control);
        float* getQuotaPointer();
        void setHomedState(bool _homed);
        bool getHomedState();
        bool isError();
    private:
        float maxStroke;
        float minStroke;
        float maxSpeed;
        float maxAcc;
        float maxTorque;
        bool homed;
        float homeQuota;
        float quota;
        JOINT_STATE state;
};

#endif