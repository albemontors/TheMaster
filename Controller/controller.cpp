#include "controller.h"
#include <cmath>
#include <cstdint>

ArmController::ArmController(u16p _MOTOR_DATA, u16p _MOTOR_SETTINGS, u16p _RESOLVER_DATA) : 
    realArm(&jointToMotor) , idealArm(&jointToMotor){

    wait_us(1000);

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
        M_param[i].CAN_READ = RESOLVER_DATA + (i * 8/2);
        M_param[i].CAN_WRITE = MOTOR_DATA + (i * 8/2);
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
    for(int i = 0; i < AXIS_COUNT; i++){
        DH[i].setParam(DH_param[i].alpha, DH_param[i].a, DH_param[i].d);
        printf("init values for axis %d are: %f %f %f \n", i, DH_param[i].alpha, DH_param[i].a, DH_param[i].d);
        printf("values in DH %d are        : %f %f %f \n", i, DH[i].alphaad[0], DH[i].alphaad[1], DH[i].alphaad[2]);
        DH[i].verbose(); }

    realArm.setTransducerArray(R);
    realArm.setDHArray(DH);
    idealArm.setJointArray(J);

    jointToMotor.init(J_param);
}

OpeMode ArmController::update(){

    if(requestedMode != NO_MODE) modeUpdate();

    switch(opeMode) {

        case DEBUG:
            r = realArm.update();
            printf("J1 = %+5.3f, J2 = %+5.3f, X = %+6.1f, Y = %+6.1f, Z = %+6.1f \n",
                r.joint.q[0],
                r.joint.q[1],
                r.cart.q.x,
                r.cart.q.y,
                r.cart.q.z);
            break;

        case NO_MODE:
            forcedMode = NO_POW;
            break;

        case NO_POW:
            // TODO: implement nopow state
            break;

        case IDLE:
            // update position of the real arm
            r = realArm.update();

            // evaluate tracking error
            trackingError = evaluateTrackingError(i.cart.q, r.cart.q);

            // update ideal position based on real position
            i = r;

            // execute inverce kinematics and update the ideal arm
            idealArm.update(i);

            break;
        
        case POWER:
            // update position of the real arm
            r = realArm.update();

            // evaluate tracking error
            trackingError = evaluateTrackingError(i.cart.q, r.cart.q);

            // hold ideal position, no update for this variables
            
            // check tracking error for values that exceed the max allowed
            if (isGreater6Df(trackingError, maxAllowedTrackingError)) { 
                forcedMode = IDLE;
                error.TRACKING_ERROR = 1;
            }
            // execute inverce kinematics and update the ideal arm
            idealArm.update(i);

            break;
    }

    
    if(!noError()) forcedMode = NO_POW;

    if(forcedMode != NO_MODE) modeUpdate();
    
    return opeMode;
}

void ArmController::modeUpdate() {
    OpeMode newOpeMode;
    OpeMode reqMode;

    if(forcedMode){
        reqMode = forcedMode;
    } else if (requestedMode){
        reqMode = requestedMode;
    }

    if(reqMode != DEBUG)
        switch(opeMode){

            case NO_MODE:
                newOpeMode = NO_POW;
                break;

            case NO_POW:
                break;

            case IDLE:
                switch(reqMode){
                    case NO_POW: newOpeMode = NO_POW; break;
                    case IDLE: if(noError()) newOpeMode = IDLE; break;
                    default: newOpeMode = IDLE;
                }
                break;

            case POWER_BEFORE_HOMED:
                switch(reqMode){
                    case NO_POW: newOpeMode = NO_POW; break;
                    case IDLE: if(noError()) newOpeMode = IDLE; break;
                    case POWER: if(noError()) /*setHome();*/ newOpeMode = POWER; break;
                }
                break;
            
            case POWER:
                switch(reqMode){
                    case NO_POW: newOpeMode = NO_POW; break;
                    case IDLE: if(noError()) newOpeMode = IDLE; break;
                }
                break;

            default:
                reqMode = NO_MODE;
        }
    else {
        newOpeMode = DEBUG;
    }

    requestedMode = NO_MODE;
    forcedMode = NO_MODE;
    opeMode = newOpeMode;
}

bool ArmController::noError() {
    if (
        error.TRACKING_ERROR == 0 &&
        error.AXIS_ERROR == 0
    ) return true;
    return false;
}

void ArmController::setHome() {
    jointToMotor.setHome();
}

void ArmController::setControlMode(ControlMode controlMode) {
    idealArm.setMovementMode(controlMode);
}

void ArmController::requestMode(OpeMode mode) {
    requestedMode = mode;
}