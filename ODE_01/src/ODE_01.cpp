#include "All.h"
#include <algorithm>

#include "ODE_01.h"

int c_main(int pargc, char** argv) {
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/yoga_gym_yoga3_1_c3d.bvh");
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/mgman__3cut4_2_x2d.bvh");
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/b_Boxer.shadow17_1_s.bvh");
//	OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/test.bvh");
		OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/david-1-martialarts-032_David.bvh");
//		OgreSimulation sim = OgreSimulation("/home/david/git/ode/ODE_01/Data/Animation/david-2cam-martialarts-006_David.bvh");
	//OgreSimulation sim = OgreSimulation();


	sim.run();
	return 0;
}


// ffi functions

OgreCanvas oc;

void initOgre() { oc.initOgre(); }
void drawBone(double startX,double startY,double startZ,double endX,double endY,double endZ, double radius) {
	oc.drawBone(Vec3(startX, startY, startZ), Vec3(endX, endY, endZ), radius);
}
void drawVec3(double originX,double originY,double originZ,double X,double Y,double Z, double radius) {
	oc.drawVec3(Vec3(originX, originY, originZ), Vec3(X, Y, Z), radius);
}
void drawPoint(double X,double Y,double Z, double radius) {
	oc.drawPoint(Vec3(X,Y,Z), radius);
}
bool doRender() {
	return oc.doRender();
}
