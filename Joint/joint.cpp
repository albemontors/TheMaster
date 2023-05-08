#include "joint.h"

Joint::Joint() {
    currentVelocity = 0.0f;
}

void Joint::setParams(J_PARAM params) {
    maxStroke = params.MAX_STROKE;
    minStroke = params.MIN_STROKE;
    maxSpeed = params.MAX_SPEED;
    maxAcc = params.MAX_ACC;
    maxTorque = params.MAX_TORQUE;
    homeQuota = params.HOME_QUOTA;
}

JOINT_CONTROL_TETRA Joint::update(JOINT_CONTROL_TETRA* control) {

    float timingFactor = 1.0f/maxAcc;

    // verify endstrokes
    if(homed) {
        if(control->controlPos > maxStroke) control->controlPos = maxStroke;
        if(control->controlPos < minStroke) control->controlPos = minStroke; }

    //save current position
    float posInput = control->controlPos;
    // evaluate position error
    float posError = posInput - currentPosition;
    // evaluate velocity error
    float velError = posError - currentVelocity;
    // calculate deceleration
    if(abs(posError) < 10.0f){
        if(currentVelocity > 0.0f) if((currentPosition + (currentVelocity * timingFactor)) > posInput) velError = -maxAcc;
        if(currentVelocity < 0.0f) if((currentPosition + (currentVelocity * timingFactor)) < posInput) velError =  maxAcc; }

    // slow down upon reaching endsrokes
    float maxSpeedPos =  maxSpeed;
    float maxSpeedNeg = -maxSpeed;
    if(homed) {
        if((currentPosition + (currentVelocity * timingFactor)) > maxStroke) velError = -maxAcc;
        if((currentPosition + (currentVelocity * timingFactor)) < minStroke) velError =  maxAcc; 
        if(maxSpeedPos >  maxSpeed) maxSpeedPos =  maxSpeed;
        if(maxSpeedPos <      0.0f) maxSpeedPos =  0.0f;
        if(maxSpeedNeg < -maxSpeed) maxSpeedNeg = -maxSpeed;
        if(maxSpeedNeg >      0.0f) maxSpeedNeg =  0.0f; }

    // limit max acceleration
    if(velError >  maxAcc) velError =  maxAcc;
    if(velError < -maxAcc) velError = -maxAcc;
    //update velocity command
    currentVelocity += (velError * (CONTROL_PERIOD/1000.0f));
    // limit max speed
    if(currentVelocity > maxSpeedPos) currentVelocity = maxSpeedPos;
    if(currentVelocity < maxSpeedNeg) currentVelocity = maxSpeedNeg;
    
    // update position command
    currentPosition += (currentVelocity * (CONTROL_PERIOD/1000.0f));
    // check endstrokes again
    if(homed) {
        if(currentPosition > maxStroke) { currentPosition = maxStroke; if(currentVelocity > 0.0f) currentVelocity = 0.0f; }
        if(currentPosition < minStroke) { currentPosition = minStroke; if(currentVelocity < 0.0f) currentVelocity = 0.0f; } }
    
    // present output
    JOINT_CONTROL_TETRA output;
    output.controlPos = currentPosition;
    output.controlVel = currentVelocity;
    output.torqueFF = control->torqueFF;
    output.integratorLimit = control->integratorLimit;

    return output;
}

float* Joint::getQuotaPointer() {
    return &quota;
}

void Joint::setHomedState(float position) {
    currentPosition = position;
    currentVelocity = 0.0f;
    homed = true;
}

void Joint::forcePosition(float position) {
    currentPosition = position;
    currentVelocity = 0.0f;
}

bool Joint::getHomedState() {
    return homed;
}

bool Joint::isError() { return state.error; }

