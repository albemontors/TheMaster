#include "canDevice.h"

Resolver::Resolver(){

}

void Resolver::setParams(M_PARAM params){
    maxSpeed = params.MAX_SPEED;
    maxTorque = params.MAX_TORQUE;
    canWrite = 0;
    canRead = params.CAN_READ;
}

MOTOR_STATE_TETRA Resolver::update() {
    MOTOR_STATE_TETRA control;

    bool overSpeed = 0;

    control.currentPos = canRead[0];
    control.currentVel = canRead[1];
    control.currentTorque = canRead[2];
    state = (State)canRead[3];

    if(control.currentVel > maxSpeed) overSpeed = 1;

    if(control.currentVel > maxTorque) overSpeed = 1;

    if(overSpeed) control.motorAxisState |= 0x10; // rise fifth bit for overspeed issue

    return control;
}

float* Resolver::getQuotaPointer(){
    return &quota;
}
