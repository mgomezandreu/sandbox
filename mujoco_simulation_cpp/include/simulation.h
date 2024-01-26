#pragma once
#include <string>

#include "mujoco/mujoco.h"

namespace Simulation{
    void load_model(mjModel** m, mjData** d, std::string model_path);
    void run_simulation(mjModel* m, mjData* d, void (*controller)(const mjModel* m, mjData* d));

    int run_simulation();
}