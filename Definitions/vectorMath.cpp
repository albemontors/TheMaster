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

VECTOR3Df evaluateTrackingError(VECTOR3Df iC, VECTOR3Df rC) {
    VECTOR3Df c;
    c = add3DF(iC, invert3DF(rC));
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