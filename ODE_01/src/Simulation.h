
#ifndef	_SIMULATION_H
#define	_SIMULATION_H	1

#include <ode/ode.h>
#include <map>

#include "Simulation.h"
#include "DynamicsMotionData.h"
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

	DynamicsMotionData md;
	map<string,dBodyID> skelBodyMap;
	map<dBodyID, string> bodySkelMap;
	map<Skeleton*, float> jointLastErrorMap;

	dWorldID wid;
	dSpaceID sid;

	vector<Skeleton*> getCurrentFrame() { return currentFrame; }
	vector<Skeleton*> getCurrentFrameFlat() {
		vector<Skeleton*> flat;
		for(Skeleton * ss : currentFrame)
			for(Skeleton * s : ss->getAllSkeletons())
				flat.push_back(s);
		return flat;
	}
	void loadFrame(int index) { currentFrame = md.getKeyframe(index); };

	// debugging / visualization functions
	virtual void drawBone(Vec3 start, Vec3 end, double radius) {};
	virtual void drawVec3(Vec3 origin, Vec3 vec, double radius = 0.05) {};
	virtual void drawPoint(Vec3 p, double radius = 0.07) {};

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

};

#endif
