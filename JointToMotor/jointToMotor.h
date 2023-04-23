#ifndef _JOINTTOMOTOR_H
#define _JOINTTOMOTOR_H

#include "definitions.h"

typedef struct {
    MOTOR_CONTROL_TETRA state[AXIS_COUNT]; }
MOTORS_COMMAND;

typedef struct {
    MOTOR_STATE_TETRA state[AXIS_COUNT]; }
MOTORS_STATE;

typedef struct {
    JOINT_CONTROL_TETRA command[AXIS_COUNT]; }
JOINTS_CONTROL;

typedef struct {
    JOINT_STATE_TRIPLET command[AXIS_COUNT];
    JOINT_STATE state[AXIS_COUNT]; }
JOINTS_STATE;

class JointToMotor{
    public:
        JointToMotor();
        void init();
        bool setHome(JOINTS_STATE input);
        MOTORS_COMMAND jtm(JOINTS_CONTROL input);
        JOINTS_STATE mtj(MOTORS_STATE input);
    private:
        float proportionalQuotas[AXIS_COUNT][AXIS_COUNT];
        float proportionalQuotasInverted[AXIS_COUNT][AXIS_COUNT];
        float homingVectorJtM[AXIS_COUNT];
        float homingVectorMtJ[AXIS_COUNT];
};

bool invert(float** M);
float getMatrixDeterminant(float** M);
float getComplementOf(float** M, int X, int Y);
void transponseMatrix(float** M);
/**
 * @note overwrites V
 */
void vecXmat(float* V, float** M);
/**
 * @note overwrites A
 */
void vecPvec(float* A, float* B);
void vecXfloat(float* A, float kb);

#endif