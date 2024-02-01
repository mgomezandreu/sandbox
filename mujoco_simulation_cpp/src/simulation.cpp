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

	// Print all information about the model
	mj_printModel(m, "model.txt");



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


	// get drone body id
	int drone_body_id = mj_name2id(m, mjOBJ_BODY, "core");

	cam.type = mjCAMERA_TRACKING;
	cam.fixedcamid = -1;
	cam.trackbodyid = drone_body_id;	
	cam.distance = 3;
	cam.azimuth = 30;
	cam.elevation = -30;
	cam.lookat[0] = 0;
	cam.lookat[1] = 0;
	cam.lookat[2] = 0.1;



	while( !glfwWindowShouldClose(window) ) {
		mjtNum simstart = d->time;
		//print drone position
		

		// set camera to follow quadrotor
		while( d->time - simstart < 1.0/60.0)
			mj_step(m, d);

		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		glfwSwapBuffers(window);

		glfwPollEvents();
	}

	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
}