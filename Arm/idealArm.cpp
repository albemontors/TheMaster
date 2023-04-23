#include "arm.h"

IdealArm::IdealArm(){
    
}

void IdealArm::setJointArray(Joint* jointArray){
    J = jointArray;
}

ControlMode IdealArm::update(ARM_CARTESIAN_VARIABLES controls) {
    ControlMode controlMode;

    return controlMode;
}