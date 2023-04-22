#include "controller.h"
#include <cmath>

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

    if(requestedMode != NO_MODE) modeUpdate();

    switch(controlMode) {

        case NO_MODE:
            requestedMode = NO_POW;
            break;

        case NO_POW:
            // TODO: implement nopow state
            break;

        case IDLE:
            // update position of the real arm
            rC = realArm.update();

            // evaluate tracking error
            trackingError = evaluateTrackingError(iC.q, rC.q);

            // update ideal position based on real position
            iC = rC;

            // execute inverce kinematics and update the ideal arm
            idealArm.update(iC);

            break;
        
        case POWER:
            // update position of the real arm
            rC = realArm.update();

            // evaluate tracking error
            trackingError = evaluateTrackingError(iC.q, rC.q);

            // hold ideal position, no update for this variables
            
            // check tracking error for values that exceed the max allowed
            if (isGreater3Df(trackingError, maxAllowedTrackingError)) { 
                forcedMode = IDLE;
                

            // execute inverce kinematics and update the ideal arm
            idealArm.update(iC);

            break;
    }

    if(forcedMode != NO_MODE) modeUpdate();
    
    return controlMode;
}

void ArmController::modeUpdate() {
    ControlMode newControlMode;
    ControlMode reqMode;

    if(forcedMode){
        reqMode = forcedMode;
    } else if (requestedMode){
        reqMode = requestedMode;
    }

    // TODO set parameters for each mode
    controlMode = reqMode;

    requestedMode = NO_MODE;
    forcedMode = NO_MODE;
    controlMode = newControlMode;
}