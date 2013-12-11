#include "All.h"

int main(int pargc, char** argv) {
	//OgreSimulation sim = OgreSimulation("Data/Animation/yoga_gym_yoga3_1_c3d.bvh");
	//OgreSimulation sim = OgreSimulation("Data/Animation/mgman__3cut4_2_x2d.bvh");
	OgreSimulation sim = OgreSimulation("Data/Animation/b_Boxer.shadow17_1_s.bvh");
	//OgreSimulation sim = OgreSimulation("Data/Animation/test.bvh");
	sim.run();
	return 0;
}
