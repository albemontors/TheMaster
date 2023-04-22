#include "canDevice.h"

Resolver::Resolver(){

}

void Resolver::setParams(M_PARAM params){
    maxSpeed = params.MAX_SPEED;
    maxTorque = params.MAX_TORQUE;
    canWrite = 0;
    canRead = params.CAN_READ;
}

MOTOR_CONTROL_TETRA Resolver::update() {
    MOTOR_CONTROL_TETRA control;

    control.controlPos = canRead[0];
    control.controlVel = canRead[1];
    control.torqueFF = canRead[2];
    state = (State)canRead[3];

    if(control.controlVel > maxSpeed); //TODO stuff

    if(control.torqueFF > maxTorque); //TODO stuff

    return control;
}

float* Resolver::getQuotaPointer(){
    return &quota;
}
