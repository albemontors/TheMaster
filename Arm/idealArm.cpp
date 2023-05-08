#include "arm.h"
#include <cstdint>

IdealArm::IdealArm(JointToMotor* jointToMotor) : Arm(jointToMotor){
    
}

void IdealArm::setJointArray(Joint* jointArray){
    J = jointArray;
}

void IdealArm::setMotorArray(Motor* motorArray){
    M = motorArray;
}

ControlMode IdealArm::update(POSE inputControls) { //$TODO

    switch(controlMode){
        case TOOL:
            // $TODO implement tool to cartesian transformation

        case CARTESIAN:
            inputControls.joint = inverseKin(inputControls);

        case JOINT:
            // validate joints positions
            JOINTS_CONTROL jointControls;
            JOINTS_CONTROL jointsOuput;
            jointsControlTranslator(&inputControls.joint, jointControls.command);
            for(int i = 0; i < AXIS_COUNT; i++) jointsOuput.command[i] = J[i].update(&(jointControls.command[i]));
            for(int i = 0; i < AXIS_COUNT; i++) if(J[i].isError()) { 
                controlMode = NO_CONTROL_MODE; 
                return controlMode; }

            controlJointsTranslator(&inputControls.joint, jointControls.command);

            // execute joint to motor conversion
            MOTORS_COMMAND motorCommand = jointToMotor->jtm(jointsOuput);

            // validate motor command and update can objects
            for(int i = 0; i < AXIS_COUNT; i++) M[i].update(motorCommand.state[i]);
            /*printf("post M1 = %5d, M2 = %5d, M3 = %5d, M4 = %5d \n\n",
                (int16_t)motorCommand.state[0].integratorLimit,
                (int16_t)motorCommand.state[1].integratorLimit,
                (int16_t)motorCommand.state[2].integratorLimit,
                (int16_t)motorCommand.state[3].integratorLimit );*/

            break;

    }

    _controls = inputControls;

    return controlMode;
}

ControlMode IdealArm::setMovementMode(ControlMode mode){ // set the control mode param, returns the new control mode (checks to be added later) //$TODO
    controlMode = mode;

    return controlMode; 
};

ControlMode IdealArm::getMovementMode(){

    return controlMode;
};

ARM_JOINTS_VARIABLES IdealArm::inverseKin(POSE pose) {

    //printf("pose  X %f, Y %f \n", pose.q.x, pose.q.y);

    vector2D scaraPose;

    float theta = atan(pose.cart.q.y / pose.cart.q.x);
    //if(theta < 0) theta += PI; // turns negative values into positive ones for PI periodic function
    scaraPose.y = (pose.cart.q.z - (L1z)) * (-1);
    scaraPose.x = (sqrt(pow((pose.cart.q.x - (L1x * cos(theta))), 2) + pow((pose.cart.q.y - (L1x * sin(theta))), 2)));
    if(pose.cart.q.x < 0.0f) scaraPose.x *= (-1.0f);



    //printf("scara X %f, Y %f \n", scaraPose.x, scaraPose.y);

    pose.joint.q[0] = theta; // assign J1 angle

    float c = sqrt(pow(scaraPose.x, 2) + pow(scaraPose.y, 2));
    if(c > L2 + L3) return pose.joint;
    float phi = atan(scaraPose.x/scaraPose.y);
    //printf("phi %f \n", phi);
    phi += PI/2;
    float beta = acos((pow(L3, 2) - pow(L2, 2) - pow(c, 2)) / ((-2.0f) * L2 * c ));
    float theta2 = phi - beta;
    float gamma = acos((pow(c, 2) - pow(L2, 2) - pow(L3, 2)) / ((-2.0f) * L2 * L3 ));
    float theta3 = -(gamma - (PI/2));
    //float theta = pose.x ? atan(pose.y / pose.x) : 90.0f; // safe atan calculation // deprecated for pose being float now

    //printf("phi %f, gamma %f, c %f \n", phi, gamma, c);

    pose.joint.q[1] = theta2;
    pose.joint.q[2] = theta3;

    return pose.joint;
}

void IdealArm::updateState(u16 requestedMode, u16 controlMode, u16 currentLimits) {
    MOTOR_SETTINGS_TETRA settings = {requestedMode, 1, controlMode, currentLimits};
    for(int i = 0; i < AXIS_COUNT; i++) M[i].updateParams(settings);
}

POSE IdealArm::getPose() {
    return _controls;
}

