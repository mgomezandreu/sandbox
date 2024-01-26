#pragma once
#include <string>

#include "mujoco/mujoco.h"

namespace Simulation{
    void run_simulation(std::string path, void (*controller)(const mjModel* m, mjData* d));
}