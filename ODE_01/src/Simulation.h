
#ifndef	_SIMULATION_H
#define	_SIMULATION_H	1

#include <ode/ode.h>
#include <map>

#include "Simulation.h"
#include "BVHParser.h"

using namespace std;

class Simulation {

public:

	Simulation(const char * bvhFile);
	~Simulation();

	void step(double dt); // step the simulation by t seconds
	vector<Skeleton*> getSkeletons();

	// is there a way to make collisionCallback private?
	void collisionCallback(dGeomID o1, dGeomID o2);

protected:

	BVHParser bvh;
	map<Skeleton*,dBodyID> skelBodyMap;

private:

	// ODE variabels
	dWorldID wid;
	dSpaceID sid;
	dJointGroupID contactGroupid;
	dJointGroupID jointGroupid;

	void initODESkeleton(Skeleton* s, dBodyID parentBodyID);
	void initODE();

	double simT;
	double odeSimT;
};

#endif
