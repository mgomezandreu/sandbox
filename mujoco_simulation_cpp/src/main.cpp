#include <iostream>
#include <GLFW/glfw3.h>

#include "mujoco/mujoco.h"

#include <numbers> 

#include "simulation.h"



void simple_constant_controller(const mjModel* m, mjData* d){
	// Apply a constant torque to the joint

	//Get sensor data for joint
	mjtNum joint_pos = d->qpos[0];
	mjtNum joint_vel = d->qvel[0];

	std::cout << "Joint position: " << joint_pos << std::endl;
	std::cout << "Joint velocity: " << joint_vel << std::endl;

	// Apply pd gain
	mjtNum kp = 2;
	mjtNum kd = 0.1;

	mjtNum joint_pos_des = 3.14 /4;

	mjtNum tau = kp * (joint_pos_des - joint_pos) + kd * (0 - joint_vel);

	d->ctrl[0] = tau;
}


int main(){

	//mjcb_control = simple_constant_controller;

	Simulation::run_simulation("../models/inverted_pendulum.xml", nullptr);
	
	return 0;
}
