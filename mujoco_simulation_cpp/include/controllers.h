#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "mujoco/mujoco.h"

namespace InvertedPendulum{
    extern mjtNum kp;
    extern mjtNum kd;
    extern mjtNum ki;
    extern mjtNum joint_pos_des;
    extern mjtNum integral;

    void control(const mjModel *m, mjData *d);
}


namespace DroneStabilization{
    void init();
    void control(const mjModel *m, mjData *d);
}

#endif // CONTROLLERS_Hm