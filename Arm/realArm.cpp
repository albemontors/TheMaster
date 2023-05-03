#include "arm.h"

RealArm::RealArm(){

}

void RealArm::setDHArray(Mat4* dhArray){
    H = dhArray;
    for(int i = 0; i < AXIS_COUNT; i++) H[i].assignTheta(&(controls.joint.q[i]));
}

void RealArm::setTransducerArray(Resolver *tArray){
    R = tArray;
}

POSE RealArm::update() { //$TODO
    POSE control;

    // collect and validate encoder data
    MOTORS_STATE motors;
    for(int i = 0; i < AXIS_COUNT; i++) motors.state[i] = R[i].update();

    // shift to joint space
    JOINTS_STATE joints = jointToMotor.mtj(motors);

    // write joints parameters
    for(int i = 0; i < AXIS_COUNT; i++) {
        control.joint.q[i] = joints.command[i].currentPos;
        control.joint.v[i] = joints.command[i].currentVel;
        control.joint.t[i] = joints.command[i].currentTorque; }

    // compute the forward kinematics
    controls.cart = forwardKin(controls.joint);

    // $TODO validate forward kinematics before preceeding

    // $todo implement tool coordinates transformation

    return control;
}

ARM_CARTESIAN_VARIABLES RealArm::forwardKin(ARM_JOINTS_VARIABLES pose){
    ARM_CARTESIAN_VARIABLES output;

    // update all DH matrices with the new theta values
    for(int i = 0; i < AXIS_COUNT; i++) H[i].update();

    // compute the multiplication through
    Mat4 cart;
    cart.write(H[0]);

    for(int i = 1; i < AXIS_COUNT; i++) cart.multiply(H[i]);

    


    return output;
}
