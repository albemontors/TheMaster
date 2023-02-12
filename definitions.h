#include "mbed.h"
#include <cstdint>
#include <stdint.h>

typedef struct { 
    uint16_t x;
    uint16_t y;
    uint16_t z; }
VECTOR3D;

typedef struct { 
    float x;
    float y;
    float z; }
VECTOR3Df;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t a;
    uint16_t b;
    uint16_t c; }
VECTOR6D;

typedef struct {
    float x;
    float y;
    float z;
    float a;
    float b;
    float c; }
VECTOR6Df;

typedef struct {
    uint16_t J1;
    uint16_t J2;
    uint16_t J3;
    uint16_t J4;
    uint16_t J5; } 
POSE;