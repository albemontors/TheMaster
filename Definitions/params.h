#ifndef _DEFINES_H
#define _DEFINES_H

#include <stdint.h>

#define AXIS_COUNT 5
#define KBRATIO (1000/6.28)

typedef struct {
    float tetha; 
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
    uint16_t* CAN_READ; }
M_PARAM;



//INIT FUNCTION
void initGlobalParam(DH_PARAM* DH_param, J_PARAM* J_param, M_PARAM* M_param);
void initJtMParam(float** mat);


#endif