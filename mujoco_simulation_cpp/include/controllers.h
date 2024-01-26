#pragma once
#include <string>

#include "mujoco/mujoco.h"

class Controller{
    public:
    virtual void control(const mjModel* m, mjData* d) = 0;
};
class InvPendPIDController : public Controller{
    public:
        InvPendPIDController();
        InvPendPIDController(mjtNum kp, mjtNum kd, mjtNum ki, mjtNum joint_pos_des);
        virtual void control(const mjModel* m, mjData* d);

    private:    
        mjtNum kp = 2;
        mjtNum kd = 0.1;
        mjtNum ki = 0.1;
        mjtNum integral = 0;
        mjtNum joint_pos_des = 3.14 /4;


};

class DroneController : public Controller{
    public:
    virtual void control(const mjModel* m, mjData* d);
};