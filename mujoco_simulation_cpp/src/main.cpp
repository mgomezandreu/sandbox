#include <iostream>

#include "mujoco/mujoco.h"


int main(){
	std::cout << "Hello Mujoco!" << std::endl;

	
	char* errstr;
	int errstr_sz;
	mjModel* m = mj_loadXML("../models/inverted_pendulum.xml", NULL, errstr, errstr_sz);

	return 0;
}
