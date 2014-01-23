
#ifndef	_SIMULATION_H
#define	_SIMULATION_H	1

#include <btBulletDynamicsCommon.h>
#include <map>

#include "Simulation.h"
#include "BVHParser.h"
#include "Constants.h"

using namespace std;

class Simulation {

public:
	Simulation();
	Simulation(const char * bvhFile);

	void step(double dt); // step the simulation by t seconds
	vector<Skeleton*> getSkeletons();

	// is there a way to make collisionCallback private?
	void collisionCallback(dGeomID o1, dGeomID o2);
	void initialOverlapCallback(dGeomID o1, dGeomID o2);

protected:

	BVHParser bvh;
	map<Skeleton*,btRigidBody*> skelBodyMap;
	map<btRigidBody*, Skeleton*> bodySkelMap;

	btDynamicsWorld * world;

	dBodyID createBall(const Vec3 & pos, const dReal & mass, const dReal & radius);

private:

	// ODE variabels
	dJointGroupID contactGroupid;
	dJointGroupID jointGroupid;

	double simT;
	double odeSimT;
	bool useBVH;

	map<dBodyID, vector<dBodyID> > overlapMap;

	void initODESkeleton(Skeleton* s, btRigidBody* parentBodyID);
	void initODE();
	bool overlap(dBodyID,dBodyID);
	void setOverlap(dBodyID,dBodyID);
};

#endif
