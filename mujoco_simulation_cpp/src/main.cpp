#include <iostream>
#include <GLFW/glfw3.h>

#include "mujoco/mujoco.h"
#include "simulation.h"
#include "controllers.h"


#include <cmath>

namespace std {
	namespace numbers {
        double pi = atan(1.0) * 4.0;
    }
}



int main(){
	
	InvPendPIDController controller(2, 0.1, 0.1, std::numbers::pi / 4);
	Simulation::run_simulation("../models/inverted_pendulum.xml", controller.control);
	
	return 0;
}
