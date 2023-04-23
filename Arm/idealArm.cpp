#include "arm.h"

IdealArm::IdealArm(){
    
}

void IdealArm::setJointArray(Joint* jointArray){
    J = jointArray;
}

ControlMode IdealArm::update(ARM_CARTESIAN_VARIABLES controls) { //$TODO
    ControlMode controlMode;

    return controlMode;
}

ControlMode IdealArm::setMovementMode(ControlMode mode){ // set the control mode param, returns the new control mode (checks to be added later) //$TODO
    controlMode = mode;

    return controlMode; 
};

ControlMode IdealArm::getMovementMode(){

    return controlMode;
};
