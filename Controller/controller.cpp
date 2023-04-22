#include "controller.h"

ArmController::ArmController(){

    // A TON OF GLOBAL VARIABLES
    DH_PARAM DH_param[AXIS_COUNT];
    J_PARAM J_param[AXIS_COUNT];
    M_PARAM M_param[AXIS_COUNT];
    //INIT ALL PARAMETERS
    initGlobalParam(DH_param, J_param, M_param);

    // init joints
    for(int i = 0; i < AXIS_COUNT; i++)
        J[i].setParams(J_param[i]);

    // init joints
    for(int i = 0; i < AXIS_COUNT; i++)
        M[i].setParams(M_param[i]);

    // init joints
    for(int i = 0; i < AXIS_COUNT; i++)
        R[i].setParams(M_param[i]);

    // init denavit hartemberg matrices
    for(int i = 0; i < AXIS_COUNT; i++)
        DH[i] = Mat4(R[i].getQuotaPointer() , DH_param[i].alpha, DH_param[i].a, DH_param[i].d);

    realArm.setTransducerArray(R);
    realArm.setHDArray(DH);
    idealArm.setJointArray(J);
}

ControlMode ArmController::update(){
    ControlMode result;


    
    return result;
}