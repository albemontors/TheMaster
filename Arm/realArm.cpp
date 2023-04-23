#include "arm.h"

RealArm::RealArm(){

}

void RealArm::setHDArray(Mat4* dhArray){
    H = dhArray;
}

void RealArm::setTransducerArray(Resolver *tArray){
    T = tArray;
}

ARM_CARTESIAN_VARIABLES RealArm::update() {
    ARM_CARTESIAN_VARIABLES control;

    return control;
}