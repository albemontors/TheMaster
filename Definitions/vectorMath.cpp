#include "definitions.h"


VECTOR3Df add3DF(VECTOR3Df a, VECTOR3Df b) {
    VECTOR3Df c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

VECTOR6Df add6DF(VECTOR6Df a, VECTOR6Df b) {
    VECTOR6Df c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    c.a = a.a + b.a;
    c.b = a.b + b.b;
    c.c = a.c + b.c;
    return c;
}

VECTOR3Df invert3DF(VECTOR3Df a) {
    VECTOR3Df c;
    c.x = -a.x;
    c.y = -a.y;
    c.z = -a.z;
    return c;
}

VECTOR6Df invert6DF(VECTOR6Df a) {
    VECTOR6Df c;
    c.x = -a.x;
    c.y = -a.y;
    c.z = -a.z;
    c.a = -a.a;
    c.b = -a.b;
    c.c = -a.c;
    return c;
}

VECTOR6Df evaluateTrackingError(VECTOR6Df iC, VECTOR6Df rC) {
    VECTOR6Df c;
    c = add6DF(iC, invert6DF(rC));
    return c;
}

bool isGreater3Df(VECTOR3Df var, VECTOR3Df param) {
    if(var.x > param.x) return true;
    if(var.y > param.y) return true;
    if(var.z > param.z) return true;
    return false;
}

bool isGreater6Df(VECTOR6Df var, VECTOR6Df param) {
    if(var.x > param.x) return true;
    if(var.y > param.y) return true;
    if(var.z > param.z) return true;
    if(var.a > param.a) return true;
    if(var.b > param.b) return true;
    if(var.c > param.c) return true;
    return false;
}

void clamp(float* var, float max, float min, float input) {
    *var += input;
    if(*var > max) *var = max;
    if(*var < min) *var = min;
}