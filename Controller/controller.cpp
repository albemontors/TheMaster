#include "controller.h"
#include <cmath>
#include <cstdint>

ArmController::ArmController(u16p _MOTOR_DATA, u16p _MOTOR_SETTINGS, u16p _RESOLVER_DATA){

    MOTOR_DATA = _MOTOR_DATA;
    MOTOR_SETTINGS = _MOTOR_SETTINGS;
    RESOLVER_DATA = _RESOLVER_DATA;

    // A TON OF GLOBAL VARIABLES
    DH_PARAM DH_param[AXIS_COUNT];
    J_PARAM J_param[AXIS_COUNT];
    M_PARAM M_param[AXIS_COUNT];
    //INIT ALL PARAMETERS
    initGlobalParam(DH_param, J_param, M_param);

    // init can links
    for(int i = 0; i < AXIS_COUNT; i++){
        M_param[i].CAN_READ = RESOLVER_DATA + (i * 4 * sizeof(uint16_t));
        M_param[i].CAN_WRITE = MOTOR_DATA + (i * 4 * sizeof(uint16_t));
    }

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

    jointToMotor.init();
}

OpeMode ArmController::update(){

    if(requestedMode != NO_MODE) modeUpdate();

    switch(opeMode) {

        case NO_MODE:
            forcedMode = NO_POW;
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
                error.TRACKING_ERROR = 1;
            }
            // execute inverce kinematics and update the ideal arm
            idealArm.update(iC);

            break;
    }

    
    if(!noError()) forcedMode = NO_POW;

    if(forcedMode != NO_MODE) modeUpdate();
    
    return opeMode;
}

void ArmController::modeUpdate() {
    OpeMode newopeMode;
    OpeMode reqMode;

    if(forcedMode){
        reqMode = forcedMode;
    } else if (requestedMode){
        reqMode = requestedMode;
    }

    switch(opeMode){
        case NO_MODE:
            newopeMode = NO_POW;
            break;

        case NO_POW:
            break;

        case IDLE:
            switch(reqMode){
                case NO_POW: newopeMode = NO_POW; break;
                case IDLE: if(noError()) newopeMode = IDLE; break;
                default: newopeMode = IDLE;
            }
            break;

        case POWER_BEFORE_HOMED:
            switch(reqMode){
                case NO_POW: newopeMode = NO_POW; break;
                case IDLE: if(noError()) newopeMode = IDLE; break;
                case POWER: if(noError()) /*setHome();*/ newopeMode = POWER; break;
            }
            break;
        
        case POWER:
            switch(reqMode){
                case NO_POW: newopeMode = NO_POW; break;
                case IDLE: if(noError()) newopeMode = IDLE; break;
            }
            break;

        default:
            reqMode = NO_MODE;
    }

    requestedMode = NO_MODE;
    forcedMode = NO_MODE;
    opeMode = newopeMode;
}

bool ArmController::noError() {
    if (
        error.TRACKING_ERROR == 0 &&
        error.AXIS_ERROR == 0
    ) return true;
    return false;
}

void ArmController::setHome() {
    // $TODO
}