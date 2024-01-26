#include <iostream>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "simulation.h"

#include <numbers>

void Simulation::run_simulation(std::string path, void (*controller)(const mjModel* m, mjData* d)){

    mjModel* m;
    mjData* d;
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(path.c_str(), NULL, error, 1000);
    if (!m) mju_error_s("Load model error: %s", error);
    d = mj_makeData(m);


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

	mjv_defaultCamera(&cam);
	//mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);


	mjv_makeScene(m, &scn, 1000);

	mjr_makeContext(m, &con, mjFONTSCALE_100);

	while( !glfwWindowShouldClose(window) ) {
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