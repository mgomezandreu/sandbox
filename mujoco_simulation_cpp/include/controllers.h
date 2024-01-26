#pragma once
#include <string>

#include "mujoco/mujoco.h"

class Controller{
    public:
    virtual void control(const mjModel* m, mjData* d) = 0;
};

class DroneController : public Controller{
    public:
    virtual void control(const mjModel* m, mjData* d);
};