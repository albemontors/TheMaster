#ifndef _DEFINES_H
#define _DEFINES_H

#include <stdint.h>

#define AXIS_COUNT 4
#define CONTROL_PERIOD 5
#define KBRATIO (1000.0f/6.28)
#define MAXMOTORSPEED 1000 // 1 rps
#define MAXMOTORTORQUE 1000 // 1 Nm
#define MAXJSPEED 2.0f // 1 rad/s
#define MAXACC 2.0f // 0.1 rads/(s*s)
#define MAXJTORQUE 1.0f // 1 Nm
#define PI 3.141593
#define L1x -13.0f
#define L1z -218.0f
#define L2 400.0f
#define L3 400.0f
#define L4 100.0f // $TODO setup L4

typedef struct {
    float alpha; 
    float a; 
    float d; }
DH_PARAM;

typedef struct {
    float MAX_STROKE;
    float MIN_STROKE;
    float MAX_SPEED;
    float MAX_ACC;
    float MAX_TORQUE;
    float HOME_QUOTA; } 
J_PARAM;

typedef struct {
    uint16_t MAX_SPEED;
    uint16_t MAX_TORQUE;
    uint16_t* CAN_WRITE;
    uint16_t* CAN_WRITE2;
    uint16_t* CAN_READ; }
M_PARAM;



//INIT FUNCTION
void initGlobalParam(DH_PARAM* DH_param, J_PARAM* J_param, M_PARAM* M_param);
void initJtMParam(float** M);


#endif