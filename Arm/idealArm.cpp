#include "arm.h"

IdealArm::IdealArm(JointToMotor* jointToMotor) : Arm(jointToMotor){
    
}

void IdealArm::setJointArray(Joint* jointArray){
    J = jointArray;
}

void IdealArm::setMotorArray(Motor* motorArray){
    M = motorArray;
}

ControlMode IdealArm::update(POSE controls) { //$TODO

    switch(controlMode){
        case TOOL:
            // $TODO implement tool to cartesian transformation

        case CARTESIAN:
            controls.joint = inverseKin(controls.cart);

        case JOINT:
            // validate joints positions
            JOINTS_CONTROL jointControls;
            jointsControlTranslator(&controls.joint, jointControls.command);
            for(int i = 0; i < AXIS_COUNT; i++) J[i].update(jointControls.command[i]);
            for(int i = 0; i < AXIS_COUNT; i++) if(J[i].isError()) { 
                controlMode = NO_CONTROL_MODE; 
                return controlMode; }

            // execute joint to motor conversion
            MOTORS_COMMAND motorCommand = jointToMotor->jtm(jointControls);

            // validate motor command and update can objects
            for(int i = 0; i < AXIS_COUNT; i++) M[i].update(motorCommand.state[i]);

            break;

    }

    return controlMode;
}

ControlMode IdealArm::setMovementMode(ControlMode mode){ // set the control mode param, returns the new control mode (checks to be added later) //$TODO
    controlMode = mode;

    return controlMode; 
};

ControlMode IdealArm::getMovementMode(){

    return controlMode;
};

ARM_JOINTS_VARIABLES IdealArm::inverseKin(ARM_CARTESIAN_VARIABLES pose) {
    ARM_JOINTS_VARIABLES output;

    return output;
}
