#include "Simulation.h"

#include <string.h>
#include <ode/ode.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "All.h"





void initialOverlapCollisionCallbackNonmemberFn(void * sim, dGeomID o1, dGeomID o2) {
	((Simulation *) sim)->initialOverlapCallback(o1, o2);
}

void collisionCallbackNonmemberFn(void * sim, dGeomID o1, dGeomID o2) {
	((Simulation *) sim)->collisionCallback(o1, o2);
}

void Simulation::initialOverlapCallback(dGeomID o1, dGeomID o2) {
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	// only look for bone on bone collisions
	if(bodySkelMap.count(b1) == 0 || bodySkelMap.count(b2) == 0 ) {
		return;
	}

	// collect collision info
	const int maxC = 1;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));

	if(c > 0) {
		setOverlap(b1,b2);
	}
}

bool Simulation::overlap(dBodyID a, dBodyID b) {
	vector<dBodyID> os = overlapMap[a];
	for(vector<dBodyID>::iterator it = os.begin(); it != os.end(); it++) {
		if(*it == b) return true;
	}
	return false;
}

void Simulation::setOverlap(dBodyID a, dBodyID b) {
	overlapMap[a].push_back(b);
	overlapMap[b].push_back(a);
}

void Simulation::collisionCallback(dGeomID o1, dGeomID o2) {
	// ignore collisions with spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		return;
	}

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	// don't collide overlapping bones
	if(b1 != 0 && b2 != 0 && overlap(b1,b2)) {
		return;
	}

//	// only plane collisions
//	if(dGeomGetClass(o1) != dPlaneClass && dGeomGetClass(o2) != dPlaneClass) {
//		return;
//	}

	// collect collision info
	const int maxC = 10;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));

	if(c > 0 && dGeomGetClass(o1) != dPlaneClass && dGeomGetClass(o2) != dPlaneClass) {
		cout << "collision between: " << bodySkelMap[b1]->name << " and " << bodySkelMap[b2]->name << endl;
		cout << contact[0].pos[0] << ", " << contact[0].pos[1] << ", " << contact[0].pos[2] << endl;
	}

	// create collision joints
	for (int i = 0; i < c; i++) {

		dContact dc;

		dc.surface.mode = dContactBounce | dContactSoftERP;
		dc.surface.soft_erp = 0.8;
		dc.surface.mu = 1;
		dc.surface.bounce = 0;
		dc.surface.bounce_vel = 0.1;
		dc.geom = contact[i];

		dJointID cj = dJointCreateContact(wid, contactGroupid, &dc);
		dJointAttach(cj, b1, b2);
	}
}

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
	int f = (int) (simT / bvh.frameTime);
	f = min(f, bvh.numFrames - 1);
	//bvh.loadKeyframe(f);

	// reflect changes in ODE
	// step ODE world in STEP_SIZE steps stopping just before current simT
	int odeSteps = (int) ((simT - odeSimT)/STEP_SIZE);
	odeSimT += odeSteps*STEP_SIZE;
	for(int i = 0; i < odeSteps; i++) {
//		dSpaceCollide(sid, this, &collisionCallbackNonmemberFn);
//		dWorldStep(wid,STEP_SIZE);
//		// Remove all joints in the contact joint group.
//		dJointGroupEmpty(contactGroupid);
	}
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
	//	Create a joint group to hold the contact joints.
	contactGroupid = dJointGroupCreate(1000);
	jointGroupid = dJointGroupCreate(100);
	/// gravity
	dWorldSetGravity(wid, 0, GRAVITY_ACC, 0);
	/// space
	sid = dHashSpaceCreate(0);
	/// floor
	dCreatePlane(sid, 0, 1, 0, -10);

	//	Set the state (position etc) of all bodies.
	for (vector<Skeleton*>::iterator ss = bvh.skeletons.begin();
			ss != bvh.skeletons.end(); ss++) {
		initODESkeleton(*ss, dBodyID());
	}

	///////
	// read the first keyframe and realize the bones
	///////
	bvh.loadKeyframe(0);

	// identify overlapping body segments
	dSpaceCollide(sid, this, &initialOverlapCollisionCallbackNonmemberFn);



//	double height = 10;
//	dBodyID bid = dBodyCreate(wid);
//	dBodySetPosition(bid,0,13,0);
//	dGeomID bGeom = dCreateCapsule(sid, 2, height);
//	dGeomSetBody(bGeom, bid);
//	dQuaternion q;
//	Vec3 pos = Vec3(0,-height,0);
//	Vec3 iDir = Vec3::UnitZ();
//	Vec3 tDir = pos.normalized();
//	Vec3 axis = iDir.cross(tDir).normalized();
//	double angle = acos(iDir.dot(tDir));
//	//dQFromAxisAndAngle(q,(dReal)axis[0],(dReal)axis[1],(dReal)axis[2],(dReal)angle);
//	Quat qEigen = (Quaterniond) AngleAxisd(angle, axis);
//	toDQuat(qEigen, q);
//	Vec3 fPos = tDir * (height/2);
//	dGeomSetOffsetPosition(bGeom, (dReal)fPos[0], (dReal)fPos[1], (dReal)fPos[2]);
//	dGeomSetOffsetQuaternion(bGeom, q);



	//	Create joints in the dynamics world.

	//	Attach the joints to the bodies.

	//	Set the parameters of all joints.

	//	Create a collision world and collision geometry objects, as necessary.
	// ??? already done ???

}

void Simulation::initODESkeleton(Skeleton* s, dBodyID parentBodyID) {

	// create a body for this bone
	dBodyID bid = dBodyCreate(wid);
//	dMass mass;
//	dMassSetSphere(&mass, 0.1, 0.1);
//	dBodySetMass(bid, &mass);

	// create a geometry for bone and attach to parent body
	if (s->hasParent()) {
		Vec3 pos = s->getPos();
		double height = pos.norm();

		// NOTE that capsules are aligned along the Z axis
		dGeomID bGeom = dCreateCapsule(sid, 3, height);
		dGeomSetBody(bGeom, parentBodyID);

		if (pos[0] == 0 && pos[1] == 0) {
			dGeomSetOffsetPosition(bGeom, 0, 0,
					((pos[2] > 0) ? 1 : -1) * height / 2);
		}
		else {
			dQuaternion q;
			Vec3 iDir = Vec3::UnitZ();
			Vec3 tDir = pos.normalized();
			Vec3 axis = iDir.cross(tDir).normalized();
			double angle = acos(iDir.dot(tDir));
			//dQFromAxisAndAngle(q,(dReal)axis[0],(dReal)axis[1],(dReal)axis[2],(dReal)angle);
			Quat qEigen = (Quaterniond) AngleAxisd(angle, axis);
			toDQuat(qEigen, q);
			Vec3 fPos = tDir * (height/2);
			dGeomSetOffsetPosition(bGeom, (dReal)fPos[0], (dReal)fPos[1], (dReal)fPos[2]);
			//dGeomSetOffsetPosition(bGeom, 0,0,1);
			dGeomSetOffsetQuaternion(bGeom, q);
		}
		// add mass to the parent
		dMass pMass;
		dBodyGetMass(parentBodyID, &pMass);
		pMass.mass += height;	// TODO have some better way of deciding mass
	}

	// set the position and orientation of the body
	Vec3 gPos = s->getPosG();
	dQuaternion gRot; toDQuat(s->getRotG(), gRot);
	dBodySetPosition(bid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);
	dBodySetQuaternion(bid, gRot);

	// attach a joint to parent
	if(s->hasParent()) {
		dJointID jid = dJointCreateBall(wid,jointGroupid);
		dJointAttach(jid,parentBodyID,bid);
		dJointSetBallAnchor(jid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);

		// append to skelBodyMap
		skelBodyMap[s] = parentBodyID;
		bodySkelMap[parentBodyID] = s;
	}

	// recurse to children
	for (vector<Skeleton*>::iterator c = s->children.begin();
			c != s->children.end(); c++) {
		initODESkeleton(*c, bid);
	}
}
