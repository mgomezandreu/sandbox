#ifndef SIMULATION_H
#define SIMULATION_H

#include <string>

#include "mujoco/mujoco.h"

namespace Simulation{
    void run_simulation(std::string path, void (*controller)(const mjModel* m, mjData* d));
}

#endif // SIMULATION_H