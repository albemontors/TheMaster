#ifndef _JOINTTOMOTOR_H
#define _JOINTTOMOTOR_H

#include "joint.h"
#include "canDevice.h"

class JointToMotor{
    public:
        JointToMotor();
        
    private:
        float proportionalQuotas[AXIS_COUNT][AXIS_COUNT];

};

#endif