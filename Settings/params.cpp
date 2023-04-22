#include "params.h"



void initGlobalParam(DH_PARAM* DH_param, J_PARAM* J_param, M_PARAM* M_param){

    DH_param[0] = {
        .tetha = 0, 
        .alpha = 0, 
        .a = 0, 
        .d = 0 
    };

    J_param[0] = {
        .J1_MAX_STROKE = 0,
        .J1_MIN_STROKE = 0,
        .J1_MAX_SPEED = 0,
        .J1_MAX_ACC = 0,
        .J1_MAX_TORQUE = 0,
        .J1_HOME_QUOTA = 0
    };

    J_param[1] = {
        .J1_MAX_STROKE = 0,
        .J1_MIN_STROKE = 0,
        .J1_MAX_SPEED = 0,
        .J1_MAX_ACC = 0,
        .J1_MAX_TORQUE = 0,
        .J1_HOME_QUOTA = 0
    };

    J_param[2] = {
        .J1_MAX_STROKE = 0,
        .J1_MIN_STROKE = 0,
        .J1_MAX_SPEED = 0,
        .J1_MAX_ACC = 0,
        .J1_MAX_TORQUE = 0,
        .J1_HOME_QUOTA = 0
    };

    J_param[3] = {
        .J1_MAX_STROKE = 0,
        .J1_MIN_STROKE = 0,
        .J1_MAX_SPEED = 0,
        .J1_MAX_ACC = 0,
        .J1_MAX_TORQUE = 0,
        .J1_HOME_QUOTA = 0
    };

    J_param[4] = {
        .J1_MAX_STROKE = 0,
        .J1_MIN_STROKE = 0,
        .J1_MAX_SPEED = 0,
        .J1_MAX_ACC = 0,
        .J1_MAX_TORQUE = 0,
        .J1_HOME_QUOTA = 0
    };

    M_param[0] = {
        .M1_MAX_SPEED = 0,
        .M1_MAX_ACC = 0,
        .M1_MAX_TORQUE = 0,
        .CAN_WRITE = 0,
        .CAN_READ = 0
    };

}