#include "jointToMotor.h"

JointToMotor::JointToMotor() {

}

void JointToMotor::init() {
    initJtMParam((float**)proportionalQuotas);
    for(int i = 0; i < AXIS_COUNT; i++) homingVector[i] = 0;

    
}

bool JointToMotor::setHome(JOINTS_STATE input) {
    for(int i = 0; i < AXIS_COUNT; i++) homingVector[i] = input.command[i].controlPos;
    return false;
}



