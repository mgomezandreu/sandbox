#include <iostream>
#include <GLFW/glfw3.h>


#include "mujoco/mujoco.h"




int vis_test(){

	char* errstr;
	int errstr_sz;

	mjModel* m = mj_loadXML("../models/inverted_pendulum.xml", NULL, errstr, errstr_sz);
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


int main(){
	std::cout << "Hello Mujoco!" << std::endl;

	
	// char* errstr;
	// int errstr_sz;
	// mjModel* m = mj_loadXML("../models/inverted_pendulum.xml", NULL, errstr, errstr_sz);
	// mjData* d = mj_makeData(m);


	// while(d->time <10){
	// 	mj_step(m,d);
	// 	std::cout << "Stepping " << d->time << std::endl;
	// }


	// mj_deleteModel(m);
	// mj_deleteData(d);

	vis_test();
	

	return 0;
}
