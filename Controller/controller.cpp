#include "controller.h"
#include <cmath>
#include <cstdint>

ArmController::ArmController(u16* _MOTOR_DATA, u16* _MOTOR_SETTINGS, u16* _RESOLVER_DATA) : 
    realArm(&jointToMotor) , idealArm(&jointToMotor){

    wait_us(1000);
    
    // A TON OF LOCAL VARIABLES
    DH_PARAM DH_param[AXIS_COUNT];
    J_PARAM J_param[AXIS_COUNT];
    M_PARAM M_param[AXIS_COUNT];

    //INIT ALL PARAMETERS
    initGlobalParam(DH_param, J_param, M_param);

    // init can links
    for(int i = 0; i < AXIS_COUNT; i++){
        M_param[i].CAN_READ = _RESOLVER_DATA + (i * 4);
        M_param[i].CAN_WRITE = _MOTOR_DATA + (i * 4);
        M_param[i].CAN_WRITE2 = _MOTOR_SETTINGS + (i * 4);
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
    idealArm.setMotorArray(M);

    jointToMotor.init(J_param);

    cartLimit[0] = {50, -300, -600};
    cartLimit[1] = {600, +300, -300};
}

OpeMode ArmController::update(INPUT input){

    POSE r;
    POSE i = idealArm.getPose();

    if(requestedMode != NO_MODE) modeUpdate();

    switch(opeMode) {

        case DEBUG:
            r = realArm.update();
            //printf("real  %f %f %f %f \n", r.joint.q[0], r.joint.q[1], r.joint.q[2], r.joint.q[3]);
            /*printf(" \nJ1 = %+5.3f, J2 = %+5.3f, J3 = %+5.3f, J4 = %+5.3f \n \nX = %+6.1f, Y = %+6.1f, Z = %+6.1f \n\n",
                r.joint.q[0],
                r.joint.q[1],
                r.joint.q[2],
                r.joint.q[3],
                r.cart.q.x,
                r.cart.q.y,
                r.cart.q.z);*/
                r.joint.q[0] = 0;
                r.joint.q[1] = 0;
                r.joint.q[2] = 0;
                r.joint.q[3] = 0;
            idealArm.setMovementMode(JOINT);
            idealArm.update(r);
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

            idealArm.setMovementMode(JOINT);

            i.joint.v[0] = 0;
            i.joint.v[1] = 0;
            i.joint.v[2] = 0;
            i.joint.v[3] = 0;

            i.joint.t[0] = 0;
            i.joint.t[1] = 0;
            i.joint.t[2] = 0;
            i.joint.t[3] = 0;

            i.joint.l[0] = 9.0f;
            i.joint.l[1] = 9.0f;
            i.joint.l[2] = 27.0f;
            i.joint.l[3] = 27.0f;

            // execute inverce kinematics and update the ideal arm
            idealArm.update(i);

            break;
        
        case POWER:
            // update position of the real arm
            r = realArm.update();

            // evaluate motor conditions
            //if(realArm.getMotorPowerState()) { forcedMode = NO_POW; break;} //DOES NOT WORK

            // evaluate tracking error
            trackingError = evaluateTrackingError(i.cart.q, r.cart.q);

            // hold ideal position, no update for this variables
            
            // check tracking error for values that exceed the max allowed
            /*if (isGreater6Df(trackingError, maxAllowedTrackingError)) { 
                forcedMode = IDLE;
                error.TRACKING_ERROR = 1;
            }*/
            // execute inverce kinematics and update the ideal arm
            idealArm.update(i);

            break;

        case MANUAL:
            // update position of the real arm
            r = realArm.update();

            // evaluate motor conditions
            //if(realArm.getMotorPowerState()) { forcedMode = NO_POW; break;} //DOES NOT WORK

            // evaluate tracking error
            trackingError = evaluateTrackingError(i.cart.q, r.cart.q);

            // apply input
            switch(idealArm.getMovementMode()) {
                case JOINT:
                    i.cart = r.cart;
                    i.joint.q[0] -= input.LY * (CONTROL_PERIOD / 1000.0f) * manJogSpeed;
                    i.joint.q[1] -= input.LX * (CONTROL_PERIOD / 1000.0f) * manJogSpeed;
                    i.joint.q[2] += input.RX * (CONTROL_PERIOD / 1000.0f) * manJogSpeed;
                    i.joint.q[3] += input.RY * (CONTROL_PERIOD / 1000.0f) * manJogSpeed;
                    break;

                case CARTESIAN:
                    //if(!inCartSpace(i.cart)) idealArm.setMovementMode(JOINT);
                    //clamp(&i.cart.q.x, cartLimit[0].q.x, cartLimit[1].q.x, input.LY * (CONTROL_PERIOD / 1000.0f) * manJogSpeed * 10.0f);
                    //clamp(&i.cart.q.y, cartLimit[0].q.y, cartLimit[1].q.y, input.LX * (CONTROL_PERIOD / 1000.0f) * manJogSpeed * 10.0f);
                    //clamp(&i.cart.q.z, cartLimit[0].q.z, cartLimit[1].q.z, input.RX * (CONTROL_PERIOD / 1000.0f) * manJogSpeed * 10.0f);
                    i.cart.q.x += input.LY * (CONTROL_PERIOD / 1000.0f) * manJogSpeed * 50.0f;
                    i.cart.q.y += input.LX * (CONTROL_PERIOD / 1000.0f) * manJogSpeed * 50.0f;
                    i.cart.q.z += input.RX * (CONTROL_PERIOD / 1000.0f) * manJogSpeed * 50.0f;
                    i.joint.q[3] += input.RY * (CONTROL_PERIOD / 1000.0f) * manJogSpeed;
                    break;
            }
            
            // check tracking error for values that exceed the max allowed
            /*if (isGreater6Df(trackingError, maxAllowedTrackingError)) { 
                forcedMode = IDLE;
                error.TRACKING_ERROR = 1;
            }*/
            // execute inverce kinematics and update the ideal arm
            idealArm.update(i);

            break;
        default:
            forcedMode = NO_POW;
    }

    
    if(!noError()) forcedMode = NO_POW;

    if(forcedMode != NO_MODE) modeUpdate();
    
    return opeMode;
}

void ArmController::modeUpdate() {
    OpeMode newOpeMode = requestedMode;
    OpeMode reqMode;

    if(forcedMode != NO_MODE){
        reqMode = forcedMode;
    } else if (requestedMode != NO_MODE){
        reqMode = requestedMode;
    }

    if(reqMode != DEBUG)
        switch(opeMode){

            case NO_MODE:
                newOpeMode = NO_POW;
                break;

            case NO_POW:
                switch(reqMode){
                    case IDLE: /*if(noError())*/ newOpeMode = IDLE; break;
                    default: newOpeMode = IDLE;
                }
                break;

            case IDLE:
                switch(reqMode){
                    case NO_POW: newOpeMode = NO_POW; break;
                    case POWER: 
                        if(noError()) newOpeMode = POWER; 
                        POSE pose = realArm.update(); 
                        for(int i = 0; i < AXIS_COUNT; i++) J[i].forcePosition(pose.joint.q[i]); 
                        break; 
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
                    case IDLE: newOpeMode = IDLE; break;
                    case MANUAL: newOpeMode = MANUAL; break;
                }
                break;

            default:
                newOpeMode = NO_POW;
        }
    else {
        newOpeMode = DEBUG;
    }

    requestedMode = NO_MODE;
    forcedMode = NO_MODE;
    opeMode = newOpeMode;
    sendMotorControlWords(opeMode);
}

bool ArmController::noError() {
    if (
        error.TRACKING_ERROR == 0 &&
        error.AXIS_ERROR == 0
    ) return true;
    return false;
}

void ArmController::setHome() {
    POSE pose = realArm.update();
    for(int i = 0; i < AXIS_COUNT; i++) J[i].setHomedState(pose.joint.q[i]);
    jointToMotor.setHome();
}

void ArmController::setControlMode(ControlMode controlMode) {
    idealArm.setMovementMode(controlMode);
}

void ArmController::requestMode(OpeMode mode) {
    requestedMode = mode;
}

OpeMode ArmController::getMode(){
    return opeMode;
}

void ArmController::setManJogSpeed(float _manJogSpeed) {
    manJogSpeed = _manJogSpeed;
}

void ArmController::sendMotorControlWords(OpeMode mode) {
    u16 requestedState = 1;
    u16 controlMode = 3;
    u16 currentLimit = CURRENT_LIMIT;
    switch(mode) {
        case NO_POW:
        case IDLE:
            requestedState = 1;
            break;
        case POWER:
        case POWER_BEFORE_HOMED:
            requestedState = 8;
            break;
        default: return;
    }
    idealArm.updateState(requestedState, controlMode, currentLimit);
    sendSettingsPdos();
}

POSE ArmController::getIdealPose() {
    return idealArm.getPose();
}

POSE ArmController::getRealPose() {
    return realArm.getPose();
}

bool ArmController::inCartSpace(ARM_CARTESIAN_VARIABLES input) {
    if(input.q.x < cartLimit[0].q.x) return false;
    if(input.q.x > cartLimit[1].q.x) return false;

    if(input.q.y < cartLimit[0].q.y) return false;
    if(input.q.y > cartLimit[1].q.y) return false;

    if(input.q.z < cartLimit[0].q.z) return false;
    if(input.q.z > cartLimit[1].q.z) return false;

    return true;
}