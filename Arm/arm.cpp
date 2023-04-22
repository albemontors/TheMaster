#include "arm.h"

Arm::Arm(){
    

    
}

void IdealArm::setJointArray(Joint* jointArray){
    J = jointArray;
}

void RealArm::setHDArray(Mat4* dhArray){
    H = dhArray;
}

void RealArm::setTransducerArray(Resolver *tArray){
    T = tArray;
}