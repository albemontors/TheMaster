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
    for(int i = 0; i < AXIS_COUNT; i++) motorCurrentMode[i] = motors.state[i].motorAxisState;
    /*printf("pre  M1 = %5d, M2 = %5d, M3 = %5d, M4 = %5d \n",
        (int16_t)motors.state[0].currentPos,
        (int16_t)motors.state[1].currentPos,
        (int16_t)motors.state[2].currentPos,
        (int16_t)motors.state[3].currentPos );*/

    // shift to joint space
    JOINTS_STATE joints = jointToMotor->mtj(motors);

    // write joints parameters
    for(int i = 0; i < AXIS_COUNT; i++) {
        _controls.joint.q[i] = joints.command[i].currentPos;
        _controls.joint.v[i] = joints.command[i].currentVel;
        _controls.joint.t[i] = joints.command[i].currentTorque; }

    // compute the forward kinematics
    _controls.cart = forwardKin(_controls.joint);

    // $TODO validate forward kinematics before preceeding

    // $todo implement tool coordinates transformation

    return _controls;
}

ARM_CARTESIAN_VARIABLES RealArm::forwardKin(ARM_JOINTS_VARIABLES pose){
    ARM_CARTESIAN_VARIABLES output;

    // update all DH matrices with the new theta values
    for(int i = 0; i < AXIS_COUNT; i++) H[i].update(pose.q[i]);

    // compute the multiplication through
    Mat4 cart;
    cart.write(H[3]);
    cart.multiply(H[2]);
    cart.multiply(H[1]);
    cart.multiply(H[0]);
/*
    Mat4 shift;
    shift.generateTrnZ(618);
    cart.multiply(shift);
*/
    output.q.x = cart.readCell(0, 3);
    output.q.y = cart.readCell(1, 3);
    output.q.z = cart.readCell(2, 3);

    return output;
}

POSE RealArm::getPose() {
    return _controls;
}

u16 RealArm::getMotorPowerState() {
    u16 output = 0;

    for(int i = 0; i < AXIS_COUNT; i++) output |= (motorCurrentMode[i] == 8) << i;
    
    return output;
}
