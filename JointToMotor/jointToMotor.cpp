#include "jointToMotor.h"

JointToMotor::JointToMotor() {

}

void JointToMotor::init() {
    initJtMParam((float**)proportionalQuotas);
    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) proportionalQuotasInverted[i][j] = proportionalQuotas[i][j];
    invert((float**)proportionalQuotasInverted);
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorMtJ[i] = 0;
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorJtM[i] = 0;
}

bool JointToMotor::setHome(JOINTS_STATE input) {
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorMtJ[i] = input.command[i].currentPos;
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorJtM[i] = homingVectorMtJ[i] * (-1);
    return false;
}

MOTORS_COMMAND JointToMotor::jtm(JOINTS_CONTROL input) {
    MOTORS_COMMAND output;
    //---------- POSITION ----------------
    float Q[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) Q[i] = input.command[i].controlPos;   
    vecPvec((float*)Q, (float*)homingVectorJtM);
    vecXmat((float*)Q, (float**)proportionalQuotas);
    vecXfloat((float*)Q, KBRATIO);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].controlPos = Q[i];
    //---------- VELOCITY ----------------
    float V[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) V[i] = input.command[i].controlVel;
    vecXmat((float*)V, (float**)proportionalQuotas);
    vecXfloat((float*)V, KBRATIO);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].controlVel = V[i];
    //---------- TORQUE ----------------
    float T[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) T[i] = input.command[i].torqueFF;
    vecXmat((float*)T, (float**)proportionalQuotasInverted);
    vecXfloat((float*)T, 1000);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].torqueFF = T[i];
    //---------- INTEGRATOR LIMIT ----------------
    float L[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) L[i] = input.command[i].integratorLimit;
    vecXmat((float*)L, (float**)proportionalQuotasInverted);
    vecXfloat((float*)L, 1000);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].integratorLimit = abs(L[i]);
    //-------------------------------------------
    return output; 
}

JOINTS_STATE JointToMotor::mtj(MOTORS_STATE input) {
    JOINTS_STATE output;
    //---------- POSITION ----------------
    float Q[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) Q[i] = input.state[i].currentPos;
    vecXfloat((float*)Q, 1.0f/KBRATIO);
    vecXmat((float*)Q, (float**)proportionalQuotasInverted);
    vecPvec((float*)Q, (float*)homingVectorMtJ);
    for(int i = 0; i < AXIS_COUNT; i++) output.command[i].currentPos = Q[i];
    //---------- VELOCITY ----------------
    float V[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) V[i] = input.state[i].currentVel;
    vecXfloat((float*)Q, 1.0f/KBRATIO);
    vecXmat((float*)V, (float**)proportionalQuotasInverted);
    for(int i = 0; i < AXIS_COUNT; i++) output.command[i].currentVel = V[i];
    //---------- TORQUE ----------------
    float T[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) T[i] = input.state[i].currentTorque;
    vecXfloat((float*)T, 0.001);
    vecXmat((float*)T, (float**)proportionalQuotas);
    for(int i = 0; i < AXIS_COUNT; i++) output.command[i].currentTorque = T[i];
    //---------- AXIS_STATE ----------------
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].isPowered = (input.state[i].motorAxisState & 0x08);
    //-------------------------------------------
    return output; 
}




