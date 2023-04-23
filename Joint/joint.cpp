#include "joint.h"

Joint::Joint() {

}

void Joint::setParams(J_PARAM params) {
    maxStroke = params.MAX_STROKE;
    minStroke = params.MIN_STROKE;
    maxSpeed = params.MAX_SPEED;
    maxAcc = params.MAX_ACC;
    maxTorque = params.MAX_TORQUE;
    homeQuota = params.HOME_QUOTA;
}

JOINT_STATE Joint::update(JOINT_CONTROL_TETRA control) {
    if(abs(control.controlPos) > maxStroke); //TODO stuff

    if(abs(control.controlPos) < minStroke); //TODO stuff

    if(abs(control.controlVel) > maxSpeed); //TODO stuff

    if(abs(control.torqueFF) > maxTorque); //TODO stuff

    return state;
}

float* Joint::getQuotaPointer() {
    return &quota;
}

void Joint::setHomedState(bool _homed) {
    homed = _homed;
}

bool Joint::getHomedState() {
    return homed;
}

