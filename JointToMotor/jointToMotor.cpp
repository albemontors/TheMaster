#include "jointToMotor.h"
#include <cstdint>

JointToMotor::JointToMotor() {

}

void JointToMotor::init(J_PARAM* jParam) {
    //initJtMParam((float**)proportionalQuotas);
    float mat[5][5] = {
        //         M1  M2  M3  M4  M5
        /*J1*/     -9,  0,  0,  0,  0,
        /*J2*/      0,  9,  0,  0,  0,
        /*J3*/      0,  0,  9,  9,  0,
        /*J4*/      0,  0, -9,  9,  0,
        /*J5*/      0,  0,  0,  0,  1, };
    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) proportionalQuotas[i][j] = mat[i][j];
    float mat2[5][5] = {
        //          M1        M2        M3        M4  M5
        /*J1*/ -1.0f/9,        0,        0,        0,  0,
        /*J2*/       0,  1.0f/9,         0,        0,  0,
        /*J3*/       0,        0,  1.0f/18, -1.0f/18,  0,
        /*J4*/       0,        0,  1.0f/18,  1.0f/18,  0,
        /*J5*/       0,        0,        0,        0,  1, };
    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) proportionalQuotasInverted[i][j] = mat2[i][j];
    //invert((float**)proportionalQuotasInverted);
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorMtJ[i] = 0;
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorJtM[i] = 0;
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorShifts[i] = jParam[i].HOME_QUOTA;
}

bool JointToMotor::setHome() {
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorJtM[i] = rawJq[i] - homingVectorShifts[i];
    for(int i = 0; i < AXIS_COUNT; i++) homingVectorMtJ[i] = homingVectorJtM[i] * (-1);
    return false;
}

MOTORS_COMMAND JointToMotor::jtm(JOINTS_CONTROL input) {
    MOTORS_COMMAND output;
    //---------- POSITION ----------------
    float Q[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) Q[i] = input.command[i].controlPos;   
    vecPvec((float*)Q, (float*)homingVectorJtM);
    vecXmat((float*)Q, proportionalQuotas);
    vecXfloat((float*)Q, KBRATIO);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].controlPos = Q[i];
    //---------- VELOCITY ----------------
    float V[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) V[i] = input.command[i].controlVel;
    vecXmat((float*)V, proportionalQuotas);
    vecXfloat((float*)V, KBRATIO);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].controlVel = V[i];
    //---------- TORQUE ----------------
    float T[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) T[i] = input.command[i].torqueFF;
    vecXmat((float*)T, proportionalQuotasInverted);
    vecXfloat((float*)T, 1000);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].torqueFF = T[i];
    //---------- INTEGRATOR LIMIT ----------------
    float L[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) L[i] = input.command[i].integratorLimit;
    vecXfloat((float*)L, 1000.0f/9.0f);
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].integratorLimit = abs(L[i]);
    //-------------------------------------------
    return output; 
}

JOINTS_STATE JointToMotor::mtj(MOTORS_STATE input) {
    JOINTS_STATE output;
    //---------- POSITION ----------------
    float Q[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) Q[i] = (int16_t)input.state[i].currentPos;
    vecXfloat((float*)Q, 1.0f/KBRATIO);
    vecXmat((float*)Q, proportionalQuotasInverted);
    for(int i = 0; i < AXIS_COUNT; i++) rawJq[i] = Q[i];
    vecPvec((float*)Q, (float*)homingVectorMtJ);
    //vecPvec((float*)Q, (float*)homingVectorShifts);
    for(int i = 0; i < AXIS_COUNT; i++) output.command[i].currentPos = Q[i];
    //---------- VELOCITY ----------------
    float V[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) V[i] = (int16_t)input.state[i].currentVel;
    vecXfloat((float*)Q, 1.0f/KBRATIO);
    vecXmat((float*)V, proportionalQuotasInverted);
    for(int i = 0; i < AXIS_COUNT; i++) output.command[i].currentVel = V[i];
    //---------- TORQUE ----------------
    float T[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) T[i] = (int16_t)input.state[i].currentTorque;
    vecXfloat((float*)T, 0.001);
    vecXmat((float*)T, proportionalQuotas);
    for(int i = 0; i < AXIS_COUNT; i++) output.command[i].currentTorque = T[i];
    //---------- AXIS_STATE ----------------
    for(int i = 0; i < AXIS_COUNT; i++) output.state[i].isPowered = (input.state[i].motorAxisState & 0x08);
    //-------------------------------------------
    return output; 
}
