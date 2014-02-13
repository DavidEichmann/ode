
#ifndef	_SIMULATION_H
#define	_SIMULATION_H	1

#include <ode/ode.h>
#include <map>

#include "Simulation.h"
#include "BVHParser.h"
#include "Constants.h"

using namespace std;

class Simulation {

public:
	Simulation();
	Simulation(const char * bvhFile);
	~Simulation();

	void step(double dt); // step the simulation by t seconds

	// is there a way to make collisionCallback private?
	void collisionCallback(dGeomID o1, dGeomID o2);
	void initialOverlapCallback(dGeomID o1, dGeomID o2);

protected:

	BVHParser bvh;
	map<string,dBodyID> skelBodyMap;
	map<dBodyID, string> bodySkelMap;
	map<Skeleton*, float> jointLastErrorMap;
	dBodyID ballID = 0;

	dWorldID wid;
	dSpaceID sid;

	dBodyID createBall(const Vec3 & pos, const dReal & mass, const dReal & radius);
	vector<Skeleton*> getCurrentFrame() { return currentFrame; }
	vector<Skeleton*> getCurrentFrameFlat() {
		vector<Skeleton*> flat;
		for(Skeleton * ss : currentFrame)
			for(Skeleton * s : ss->getAllSkeletons())
				flat.push_back(s);
		return flat;
	}
	void loadFrame(int index) { currentFrame = bvh.getKeyframe(index); };

private:
	// ODE variabels
	dJointGroupID contactGroupid;
	dJointGroupID jointGroupid;

	double simT;
	double odeSimT;
	bool useBVH;

	map<dBodyID, vector<dBodyID> > overlapMap;

	vector<Skeleton*> currentFrame;

	void initODESkeleton(Skeleton* s, dBodyID parentBodyID);
	void initODE();
	bool overlap(dBodyID,dBodyID);
	void setOverlap(dBodyID,dBodyID);


	dReal impulseF = 0;
	int stepsLeft = 0;
	int impulsSteps = max(1,(int) (3 / STEP_SIZE));
	int restSteps = max(1,(int) (0.5 / STEP_SIZE));
	void controlBall(dBodyID ballID, dReal t, dReal dt);
};

#endif
