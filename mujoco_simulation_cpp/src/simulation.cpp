#include <iostream>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "simulation.h"

#include <numbers>

void Simulation::load_model(mjModel** m, mjData** d, std::string model_path){
    // Load model from file and check for errors
    char error[1000] = "Could not load binary model";
    *m = mj_loadXML(model_path.c_str(), NULL, error, 1000);
    if (!*m) mju_error_s("Load model error: %s", error);

    // Make data corresponding to model
    *d = mj_makeData(*m);
}

void Simulation::run_simulation(mjModel* m, mjData* d, void (*controller)(const mjModel* m, mjData* d)){

    if(!m || !d){
        std::cout << "Model or data not loaded" << std::endl;
        return;
    }

    if (controller != nullptr){
        mjcb_control = controller;
    } else{
        std::cout << "No controller provided. Falling back to default 0 controller" << std::endl;
    }

    mjvCamera cam;                      // abstract camera
	mjvOption opt;                      // visualization options
	mjvScene scn;                       // abstract scene
	mjrContext con; 


	glfwInit();
	GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	//mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 1000);
	mjr_makeContext(m, &con, mjFONTSCALE_100);

	// ... install GLFW keyboard and mouse callbacks

	// run main loop, target real-time simulation and 60 fps rendering
	while( !glfwWindowShouldClose(window) ) {
	// advance interactive simulation for 1/60 sec
	//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
	//  this loop will finish on time for the next frame to be rendered at 60 fps.
	//  Otherwise add a cpu timer and exit this loop when it is time to render.
		mjtNum simstart = d->time;
		while( d->time - simstart < 1.0/60.0 )
			mj_step(m, d);

		// get framebuffer viewport
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// update scene and render
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(window);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}

	// close GLFW, free visualization storage
	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

    

}

int Simulation::run_simulation(){

	char* errstr;
	int errstr_sz;

	mjModel* m = mj_loadXML("../models/quadrotor_plus.xml", NULL, errstr, errstr_sz);
	mjData* d = mj_makeData(m);
	mjvCamera cam;                      // abstract camera
	mjvOption opt;                      // visualization options
	mjvScene scn;                       // abstract scene
	mjrContext con;                     // custom GPU context

	// ... load model and data

	// init GLFW, create window, make OpenGL context current, request v-sync
	glfwInit();
	GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	//mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 1000);
	mjr_makeContext(m, &con, mjFONTSCALE_100);

	// ... install GLFW keyboard and mouse callbacks

	// run main loop, target real-time simulation and 60 fps rendering
	while( !glfwWindowShouldClose(window) ) {
	// advance interactive simulation for 1/60 sec
	//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
	//  this loop will finish on time for the next frame to be rendered at 60 fps.
	//  Otherwise add a cpu timer and exit this loop when it is time to render.
		mjtNum simstart = d->time;
		while( d->time - simstart < 1.0/60.0 )
			mj_step(m, d);

		// get framebuffer viewport
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// update scene and render
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(window);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}

	// close GLFW, free visualization storage
	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

	return 0;

}