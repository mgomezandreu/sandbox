#include <iostream>
#include <GLFW/glfw3.h>

#include "mujoco/mujoco.h"
#include "simulation.h"
#include "controllers.h"


int main(){

	InvertedPendulum::kp = 100;
	InvertedPendulum::kd = 10;
	InvertedPendulum::ki = 0;
	InvertedPendulum::joint_pos_des = 0.5;
	InvertedPendulum::integral = 0;

	Simulation::run_simulation(
			"../models/inverted_pendulum.xml",
			InvertedPendulum::control);
	return 0;
}
