#ifndef _DEFINES_H
#define _DEFINES_H

#define AXIS_COUNT 5



typedef struct {
    float tetha; 
    float alpha; 
    float a; 
    float d; }
DH_PARAM;

typedef struct {
    float J1_MAX_STROKE;
    float J1_MIN_STROKE;
    float J1_MAX_SPEED;
    float J1_MAX_ACC;
    float J1_MAX_TORQUE;
    float J1_HOME_QUOTA; } 
J_PARAM;

typedef struct {
    float M1_MAX_SPEED;
    float M1_MAX_ACC;
    float M1_MAX_TORQUE;
    int* CAN_WRITE;
    int* CAN_READ; }
M_PARAM;



//INIT FUNCTION
void initGlobalParam(DH_PARAM* DH_param, J_PARAM* J_param, M_PARAM* M_param);


#endif