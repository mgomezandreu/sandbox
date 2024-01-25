#include <iostream>

#include "mujoco/mujoco.h"


int main(){
	std::cout << "Hello Mujoco!" << std::endl;

	
	char* errstr;
	int errstr_sz;
	mjModel* m = mj_loadXML("../models/inverted_pendulum.xml", NULL, errstr, errstr_sz);
	mjData* d = mj_makeData(m);


	while(d->time <10){
		mj_step(m,d);
		std::cout << "Stepping " << d->time << std::endl;
	}


	mj_deleteModel(m);
	mj_deleteData(d);
	

	return 0;
}
