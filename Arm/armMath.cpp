#include "arm.h"

void jointsControlTranslator(ARM_JOINTS_VARIABLES* armJoints, JOINT_CONTROL_TETRA* jointsControls) {
    for(int i = 0; i < AXIS_COUNT; i++){
        jointsControls[i].controlPos =  armJoints->q[i];
        jointsControls[i].controlVel =  armJoints->v[i];
        jointsControls[i].torqueFF =  armJoints->t[i];
        jointsControls[i].integratorLimit =  armJoints->l[i]; }
}