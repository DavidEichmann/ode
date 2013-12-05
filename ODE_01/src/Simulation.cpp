
#include "Simulation.h"

#include <string.h>
#include <ode/ode.h>

#include "All.h"

const double Simulation::STEP_SIZE = STEP_SIZE;

Simulation::Simulation(const char * bvhFile) {
	simT = 0;

	// parse input file
	bvh.parse(bvhFile);

	// setup ODE
	initODE();
}

Simulation::~Simulation() {
	dWorldDestroy(wid);
}

void Simulation::step(double dt) {

	// load next keyframe according to time delta
	simT += dt;
	int f = (int) (simT/bvh.frameTime);
	f = min(f, bvh.numFrames-1);
	bvh.loadKeyframe(f);

	// reflect changes in ODE
	// step ODE world
	// update SceneGraph from ODE ???? or just leave ??? depends on how keyframes are refelected in ODE world
	// TODO

}

vector<Skeleton*> Simulation::getSkeletons() {
	return bvh.skeletons;
}

void Simulation::initODE() {
	dInitODE();
	//	Create a dynamics world.
	wid = dWorldCreate();
	/// gravity
	dWorldSetGravity(wid, 0, GRAVITY_ACC, 0);
	/// space
	sid = dHashSpaceCreate(0);
	/// floor
	dCreatePlane(sid, 0, 1, 0, 0);

//	/// simple sphere
//	bid = dBodyCreate(wid);
//	static dGeomID gid = dCreateSphere(space, 1);
//	static dMass mass;
//	dMassSetSphere(&mass, 1, 1);
//	dBodySetMass(bid, &mass);
//	dGeomSetBody(gid, bid);

	///////
	// read the scene graph and realize the bones
	///////

	//	Set the state (position etc) of all bodies.

	//	Create joints in the dynamics world.

	//	Attach the joints to the bodies.

	//	Set the parameters of all joints.

	//	Create a collision world and collision geometry objects, as necessary.
	// ??? already done ???

	//	Create a joint group to hold the contact joints.
	contactGroupid = dJointGroupCreate(1000);

}
