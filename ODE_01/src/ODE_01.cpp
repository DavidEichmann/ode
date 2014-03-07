#include "All.h"
#include <algorithm>

#include "ODE_01.h"

int c_main(int pargc, char** argv) {
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/yoga_gym_yoga3_1_c3d.bvh");
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/mgman__3cut4_2_x2d.bvh");
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/b_Boxer.shadow17_1_s.bvh");
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/test.bvh");
		OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-005_David.bvh");
//		OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/david-2cam-martialarts-006_David.bvh");
	//OgreSimulation sim = OgreSimulation();


	sim.run();
	return 0;
}

