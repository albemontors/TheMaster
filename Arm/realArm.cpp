#include "arm.h"

RealArm::RealArm(JointToMotor* jointToMotor) : Arm(jointToMotor){

}

void RealArm::setDHArray(Mat4* dhArray){
    H = dhArray;
}

void RealArm::setTransducerArray(Resolver *tArray){
    R = tArray;
}

POSE RealArm::update() { //$TODO

    // collect and validate encoder data
    MOTORS_STATE motors;
    for(int i = 0; i < AXIS_COUNT; i++) motors.state[i] = R[i].update();

    // shift to joint space
    JOINTS_STATE joints = jointToMotor->mtj(motors);

    // write joints parameters
    for(int i = 0; i < AXIS_COUNT; i++) {
        controls.joint.q[i] = joints.command[i].currentPos;
        controls.joint.v[i] = joints.command[i].currentVel;
        controls.joint.t[i] = joints.command[i].currentTorque; }

    // compute the forward kinematics
    controls.cart = forwardKin(controls.joint);

    // $TODO validate forward kinematics before preceeding

    // $todo implement tool coordinates transformation

    return controls;
}

ARM_CARTESIAN_VARIABLES RealArm::forwardKin(ARM_JOINTS_VARIABLES pose){
    ARM_CARTESIAN_VARIABLES output;

    // update all DH matrices with the new theta values
    for(int i = 0; i < AXIS_COUNT; i++) H[i].update(pose.q[i]);

    // compute the multiplication through
    Mat4 cart;
    cart.write(H[1]);
    cart.multiply(H[0]);

    Mat4 shift;
    shift.generateTrnZ(618);
    cart.multiply(shift);

    output.q.x = cart.readCell(0, 3);
    output.q.y = cart.readCell(1, 3);
    output.q.z = cart.readCell(2, 3);

    return output;
}

POSE RealArm::getPose() {
    return controls;
}
