#include "canDevice.h"
#include <cstdint>

Motor::Motor(){

}
    
void Motor::setParams(M_PARAM params){
    maxSpeed = params.MAX_SPEED;
    maxTorque = params.MAX_TORQUE;
    canWrite = params.CAN_WRITE;
    canWrite2 = params.CAN_WRITE2;
    canRead = 0;
}

void Motor::update(MOTOR_CONTROL_TETRA control){
    //if(abs((int16_t)control.controlVel) > maxSpeed) return; //TODO stuff

    //if(abs((int16_t)control.torqueFF) > maxTorque) return; //TODO stuff

    canWrite[0] = control.controlPos; 
    canWrite[1] = control.controlVel;
    canWrite[2] = control.torqueFF;
    canWrite[3] = control.integratorLimit;

    return;
}

void Motor::updateParams(MOTOR_SETTINGS_TETRA settings) {
    
    canWrite2[0] = settings.requestedMode;
    canWrite2[1] = settings.inputMode;
    canWrite2[2] = settings.controlMode;
    canWrite2[3] = settings.currentLimit;

}
