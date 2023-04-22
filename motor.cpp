#include "canDevice.h"

Motor::Motor(){

}
    
void Motor::setParams(M_PARAM params){
    maxSpeed = params.MAX_SPEED;
    maxTorque = params.MAX_TORQUE;
    canWrite = params.CAN_WRITE;
    canRead = 0;
}

void Motor::update(MOTOR_CONTROL_TETRA control){
    if(control.controlVel > maxSpeed) return; //TODO stuff

    if(control.torqueFF > maxTorque) return; //TODO stuff

    canWrite[0] = control.controlPos;
    canWrite[1] = control.controlVel;
    canWrite[2] = control.torqueFF;
    canWrite[3] = control.integratorLimit;

    return;
}
