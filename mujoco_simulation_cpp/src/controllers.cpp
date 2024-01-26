#include "controllers.h"

InvPendPIDController::InvPendPIDController(){
    // Default constructor
}
InvPendPIDController::InvPendPIDController(mjtNum kp, mjtNum kd, mjtNum ki, mjtNum joint_pos_des){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    this->joint_pos_des = joint_pos_des;
}

void InvPendPIDController::control(const mjModel* m, mjData* d){

    mjtNum joint_pos = d->qpos[0];
    mjtNum joint_vel = d->qvel[0];


    mjtNum tau = kp * (joint_pos_des - joint_pos) + kd * (0 - joint_vel) + ki * integral;

    integral += joint_pos_des - joint_pos;

    d->ctrl[0] = tau;
}