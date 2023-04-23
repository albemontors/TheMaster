#include "params.h"

void initGlobalParam(DH_PARAM* DH_param, J_PARAM* J_param, M_PARAM* M_param){

    DH_param[0] = {
        .tetha = 0, 
        .alpha = 0, 
        .a = 0, 
        .d = 0 
    };

    J_param[0] = {
        .MAX_STROKE = 0,
        .MIN_STROKE = 0,
        .MAX_SPEED = 0,
        .MAX_ACC = 0,
        .MAX_TORQUE = 0,
        .HOME_QUOTA = 0
    };

    J_param[1] = {
        .MAX_STROKE = 0,
        .MIN_STROKE = 0,
        .MAX_SPEED = 0,
        .MAX_ACC = 0,
        .MAX_TORQUE = 0,
        .HOME_QUOTA = 0
    };

    J_param[2] = {
        .MAX_STROKE = 0,
        .MIN_STROKE = 0,
        .MAX_SPEED = 0,
        .MAX_ACC = 0,
        .MAX_TORQUE = 0,
        .HOME_QUOTA = 0
    };

    J_param[3] = {
        .MAX_STROKE = 0,
        .MIN_STROKE = 0,
        .MAX_SPEED = 0,
        .MAX_ACC = 0,
        .MAX_TORQUE = 0,
        .HOME_QUOTA = 0
    };

    J_param[4] = {
        .MAX_STROKE = 0,
        .MIN_STROKE = 0,
        .MAX_SPEED = 0,
        .MAX_ACC = 0,
        .MAX_TORQUE = 0,
        .HOME_QUOTA = 0
    };

    M_param[0] = {
        .MAX_SPEED = 0,
        .MAX_TORQUE = 0,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };

}

void initJtMParam(float** mat) {
    float init[AXIS_COUNT][AXIS_COUNT] = {

    //         M1  M2  M3  M4  M5
    /*J1*/      9,  0,  0,  0,  0,
    /*J2*/      0,  9,  0,  0,  0,
    /*J3*/      0,  0,  9,  9,  0,
    /*J4*/      0,  0, -9,  9,  0,
    /*J5*/      0,  0,  0,  0,  1,

    };

    float kbRatio = 1000/6.28;

    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) init[i][j] *= kbRatio;

    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) mat[i][j] = init[i][j];
}