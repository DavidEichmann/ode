
#ifndef	_SIMULATION_H
#define	_SIMULATION_H	1

#include <ode/ode.h>

#include "Simulation.h"
#include "BVHParser.h"

using namespace std;

class Simulation {

public:

	const static double STEP_SIZE;

	Simulation(const char * bvhFile);
	~Simulation();

	void step(double dt); // step the simulation by t seconds
	vector<Skeleton*> getSkeletons();

protected:

	BVHParser bvh;

private:

	// ODE variabels
	dWorldID wid;
	dSpaceID sid;
	dJointGroupID contactGroupid;

	void loadSkeletons(vector<Skeleton*> ss);
	void initODE();

	double simT;
};

#endif
