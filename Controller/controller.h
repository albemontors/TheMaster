#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm.h"

class ArmController{
    public:
        ArmController();
        ControlMode update();
    private:
        Joint J[AXIS_COUNT];
        Motor M[AXIS_COUNT];
        Mat4 DH[AXIS_COUNT];
        Resolver R[AXIS_COUNT];
        JointToMotor jointToMotor;
        RealArm realArm;
        IdealArm idealArm;
        float k;
        VECTOR3Df qCr;
        VECTOR3Df vCr;
        VECTOR3Df qCi;
        VECTOR3Df vCi;
        VECTOR3Df trackingError;
        DH_PARAM DH_param[AXIS_COUNT];
        J_PARAM J_param[AXIS_COUNT];
        M_PARAM M_param[AXIS_COUNT];
        ControlMode controlMode;
};

#endif