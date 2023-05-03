#include "params.h"

#define MAXMOTORSPEED 1000 // 1 rps
#define MAXMOTORTORQUE 1000 // 1 Nm
#define MAXJSPEED 1.0f // 1 rad/s
#define MAXJTORQUE 1.0f // 1 Nm
#define PI 3.141593
#define L1x -218
#define L1z -13
#define L2 400
#define L3 400
#define L4 100 // $TODO setup L4

void initGlobalParam(DH_PARAM* DH_param, J_PARAM* J_param, M_PARAM* M_param){

    DH_param[0] = {
        .alpha = PI/2, 
        .a = L1x, 
        .d = L1z 
    };
    J_param[0] = {
        .MAX_STROKE = PI/2,
        .MIN_STROKE = -PI/2,
        .MAX_SPEED = MAXJSPEED,
        .MAX_ACC = 0,
        .MAX_TORQUE = MAXJTORQUE,
        .HOME_QUOTA = 0
    };
    M_param[0] = {
        .MAX_SPEED = MAXMOTORSPEED,
        .MAX_TORQUE = MAXMOTORTORQUE,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };
#if AXIS_COUNT >= 2
    DH_param[1] = {
        .alpha = 0, 
        .a = -L2, 
        .d = 0 
    };
    J_param[1] = {
        .MAX_STROKE = 5*(PI/6),
        .MIN_STROKE = PI/6,
        .MAX_SPEED = MAXJSPEED,
        .MAX_ACC = 0,
        .MAX_TORQUE = MAXJTORQUE,
        .HOME_QUOTA = PI/2
    };
    M_param[1] = {
        .MAX_SPEED = MAXMOTORSPEED,
        .MAX_TORQUE = MAXMOTORTORQUE,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };
#endif
#if AXIS_COUNT >= 3
    DH_param[2] = {
        .alpha = -PI/2, 
        .a = 0, 
        .d = 0 
    };
    J_param[2] = {
        .MAX_STROKE = 0,
        .MIN_STROKE = -PI, // $TODO refine endstrokes
        .MAX_SPEED = MAXJSPEED,
        .MAX_ACC = 0,
        .MAX_TORQUE = MAXJTORQUE,
        .HOME_QUOTA = -PI/2
    };
    M_param[2] = {
        .MAX_SPEED = MAXMOTORSPEED,
        .MAX_TORQUE = MAXMOTORTORQUE,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };
#endif
#if AXIS_COUNT >= 4
    DH_param[3] = {
        .alpha = PI/2, 
        .a = 0, 
        .d = -L3 
    };
    J_param[3] = {
        .MAX_STROKE = PI,
        .MIN_STROKE = -PI,
        .MAX_SPEED = MAXJSPEED,
        .MAX_ACC = 0,
        .MAX_TORQUE = MAXJTORQUE,
        .HOME_QUOTA = 0
    };
    M_param[3] = {
        .MAX_SPEED = MAXMOTORSPEED,
        .MAX_TORQUE = MAXMOTORTORQUE,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };
#endif
#if AXIS_COUNT >= 5
    DH_param[4] = {
        .alpha = -PI/2, 
        .a = L4, 
        .d = 0 
    };
    J_param[4] = {
        .MAX_STROKE = 0, // $TODO set J5 params
        .MIN_STROKE = 0,
        .MAX_SPEED = 0,
        .MAX_ACC = 0,
        .MAX_TORQUE = 0,
        .HOME_QUOTA = 0
    };
    M_param[4] = {
        .MAX_SPEED = MAXMOTORSPEED,
        .MAX_TORQUE = MAXMOTORTORQUE,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };
#endif
    

}


void initJtMParam(float** mat) {

#if AXIS_COUNT == 5
    float init[AXIS_COUNT][AXIS_COUNT] = {
    //         M1  M2  M3  M4  M5
    /*J1*/      9,  0,  0,  0,  0,
    /*J2*/      0,  9,  0,  0,  0,
    /*J3*/      0,  0,  9,  9,  0,
    /*J4*/      0,  0, -9,  9,  0,
    /*J5*/      0,  0,  0,  0,  1,
    };
#endif

#if AXIS_COUNT == 2
    float init[AXIS_COUNT][AXIS_COUNT] = {
    //         M1  M2
    /*J1*/      9,  0,
    /*J2*/      0,  9,
    };
#endif



    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) mat[i][j] = init[i][j];
}

