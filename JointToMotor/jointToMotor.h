#ifndef _JOINTTOMOTOR_H
#define _JOINTTOMOTOR_H

#include "definitions.h"

typedef struct {
    MOTOR_STATE_TETRA state[AXIS_COUNT]; }
MOTORS_COMMAND;

typedef struct {
    JOINT_CONTROL_TETRA command[AXIS_COUNT];
    JOINT_STATE state[AXIS_COUNT]; }
JOINTS_STATE;

class JointToMotor{
    public:
        JointToMotor();
        void init();
        bool setHome(JOINTS_STATE input);
        MOTORS_COMMAND jtm(JOINTS_STATE input);
        JOINTS_STATE mtj(MOTORS_COMMAND input);
    private:
        float proportionalQuotas[AXIS_COUNT][AXIS_COUNT];
        float proportionalQuotasInverted[AXIS_COUNT][AXIS_COUNT];
        float homingVector[AXIS_COUNT];
};

#endif