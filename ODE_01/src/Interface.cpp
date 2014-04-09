#include "OgreCanvas.h"
#include <Eigen/Core>
#include "Util.h"

#include "Interface.h"

// ffi functions

OgreCanvas oc;

void initOgre() { oc.initOgre(); }
void drawBone(double r,double g,double b,double a,   double startX,double startY,double startZ,double endX,double endY,double endZ, double radius) {
	oc.drawBone(Ogre::ColourValue(r,g,b,a), Vec3(startX, startY, startZ), Vec3(endX, endY, endZ), radius);
}
void drawVec3(double r,double g,double b,double a,   double originX,double originY,double originZ,double X,double Y,double Z, double radius) {
	oc.drawVec3(Ogre::ColourValue(r,g,b,a), Vec3(originX, originY, originZ), Vec3(X, Y, Z), radius);
}
void drawPoint(double r,double g,double b,double a,   double X,double Y,double Z, double radius) {
	oc.drawPoint(Ogre::ColourValue(r,g,b,a), Vec3(X,Y,Z), radius);
}
bool doRender() {
	return oc.doRender();
}



dWorldID wid;
dSpaceID sid;
dJointGroupID jointGroupid;
dJointGroupID contactGroupid;
double * bodyPosRot = new double[3+4]; // pos + rot


double * getBodyPosRot(dBodyID bid) {
	const dReal * pos = dBodyGetPosition(bid);
	const dReal * rot = dBodyGetQuaternion(bid);
	memcpy(bodyPosRot,		pos,	3 * sizeof(double));
	memcpy(bodyPosRot + 3,	rot,	4 * sizeof(double));
	return bodyPosRot;
}

void setBodyPosRot(dBodyID bid, double x, double y, double z, double qw, double qx, double qy, double qz) {
	dBodySetPosition(bid,x,y,z);
	dQuaternion q{qw, qx, qy, qz};
	dBodySetQuaternion(bid,q);
}

double* getBodyStartEnd(dBodyID bid) {

	dGeomID g = dBodyGetFirstGeom(bid);
	const dReal * dq = dBodyGetQuaternion(bid);
	dReal doq[4]; dGeomGetOffsetQuaternion(g,doq);

	dReal radius;
	dReal length;
	dGeomCapsuleGetParams(g,&radius,&length);

	Quat qoq = eigQuat(dq) * eigQuat(doq);
	Vec3 pos = eigVec3(dBodyGetPosition(bid));

	Vec3 boneVec = qoq * Vec3(0,0,length/2);
	Vec3 start = pos + boneVec;
	Vec3 end   = pos - boneVec;

	bodyPosRot[0] = start[0];
	bodyPosRot[1] = start[1];
	bodyPosRot[2] = start[2];
	bodyPosRot[3] = end[0];
	bodyPosRot[4] = end[1];
	bodyPosRot[5] = end[2];
	return bodyPosRot;
}

dWorldID initODE() {
	dInitODE();
	//	Create a dynamics world.
	wid = dWorldCreate();
	// set CFM
	dWorldSetCFM(wid,0.00007);
	dWorldSetERP(wid,0.1);
	//	Create a joint group to hold the contact joints.
	jointGroupid = dJointGroupCreate(100);
	contactGroupid = dJointGroupCreate(1000);
	/// gravity
	dWorldSetGravity(wid, 0, -GRAVITY_ACC, 0);
	/// space
	sid = dHashSpaceCreate(0);
	/// floor
	dCreatePlane(sid, 0, 1, 0, 0);

	return wid;
}

dBodyID appendCapsuleBody(

		// position CoM
		double x, double y, double z,

		// local rotation for the bone from Z aligned
		double qw, double qx, double qy, double qz,
		// global quaternion rotation
		double rqw, double rqx, double rqy, double rqz,

		// dimensions
		double radius,
		double length,

		// mass
		double mass,

		// Inertia matrix (about CoM)
		double i11, double i12, double i13,
					double i22, double i23,
								double i33

) {

	// create a body for this bone
	dBodyID bid = dBodyCreate(wid);

	// NOTE that capsules are aligned along the Z axis
	dGeomID bGeom = dCreateCapsule(sid, radius, length);
	dGeomSetBody(bGeom, bid);

	// set the local rotation and offset
	dQuaternion q{qw,qx,qy,qz};
	dGeomSetOffsetQuaternion(bGeom, q);

	// add mass to the parent
	dMass pMass;
	dBodyGetMass(bid, &pMass);
	dMassSetParameters (&pMass, mass,
							 0,0,0,
							 i11, i22, i33,
							 i12, i13, i23);
	dBodySetMass(bid, &pMass);

	// set the position and orientation of the body
	Vec3 gPos(x,y,z);
	dQuaternion gRot{rqw,rqx,rqy,rqz};
	dBodySetPosition(bid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);
	dBodySetQuaternion(bid, gRot);


	return bid;
}

void createBallJoint(dBodyID a, dBodyID b, double x, double y, double z) {
	dJointID jid = dJointCreateBall(wid,jointGroupid);
	dJointAttach(jid, a, b);
	dJointSetBallAnchor(jid, x, y, z);
}

// we use 3 axies: one to enforce the magnatude of angular velocity about the given axis,
// and the other 2 to constrain angular velocity to the angular velocity axis
dJointID createAMotor(dBodyID a, dBodyID b) {
	dJointID jid = dJointCreateAMotor(wid, jointGroupid);
	dJointAttach(jid, a, b);
	dJointSetAMotorMode(jid, dAMotorUser);
	dJointSetAMotorNumAxes(jid, 3);
	dJointSetAMotorParam(jid, dParamFMax,  dInfinity);
	dJointSetAMotorParam(jid, dParamFMax1, dInfinity);
	dJointSetAMotorParam(jid, dParamFMax2, dInfinity);
	dJointSetAMotorParam(jid, dParamFMax3, dInfinity);
	return jid;
}

void setAMotorVelocity(dJointID jid, double x, double y, double z) {
	if(x == 0 && y == 0 && z == 0) {
		dJointSetAMotorAxis(jid,0,0, 1,0,0);
		dJointSetAMotorAxis(jid,1,0, 0,1,0);
		dJointSetAMotorAxis(jid,2,0, 0,0,1);
		dJointSetAMotorParam(jid, dParamVel, 0);
		dJointSetAMotorParam(jid, dParamVel1, 0);
		dJointSetAMotorParam(jid, dParamVel2, 0);
		dJointSetAMotorParam(jid, dParamVel3, 0);
	}
	else {
		// axis 0 is the rotation axis
		Vec3 a1{x,y,z};
		dJointSetAMotorAxis(jid,0,0, (double) a1(0), (double) a1(1), (double) a1(2));
		dJointSetAMotorParam(jid, dParamVel1, a1.norm());
		// axis 1 and 2 are axies perpendicular to axis 0 such that all axis are orthogonal
		Vec3 a2;
		Vec3 a3;
		if(x == 0 && y == 0) {
			a2 = Vec3{1,0,0};
		}
		else {
			a2 = a1.cross(Vec3{x,y,z+1});
		}
		a3 = a1.cross(a2);
		dJointSetAMotorAxis(jid,1,0, (double) a2(0), (double) a2(1), (double) a2(2));
		dJointSetAMotorParam(jid, dParamVel2, 0);
		dJointSetAMotorAxis(jid,2,0, (double) a3(0), (double) a3(1), (double) a3(2));
		dJointSetAMotorParam(jid, dParamVel3, 0);
	}
}

void createFixedJoint(dBodyID a, dBodyID b) {
	dJointID jid = dJointCreateFixed(wid,jointGroupid);
	dJointAttach(jid, a, b);
	dJointSetFixed(jid);
}

void collisionCallback(void * sim, dGeomID o1, dGeomID o2) {

	// ignore collisions with spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		return;
	}

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

//	// don't collide overlapping/connected bones
//	if(b1 != 0 && b2 != 0 && (dAreConnectedExcluding(b1,b2,dJointTypeContact) || overlap(b1,b2))) {
//		return;
//	}

	// only plane collisions
	if(dGeomGetClass(o1) != dPlaneClass && dGeomGetClass(o2) != dPlaneClass) {
		return;
	}

	// collect collision info
	const int maxC = 10;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));

	// create collision joints
	for (int i = 0; i < c; i++) {
		if(contact[i].depth != 0) {
			dContact dc;

			dc.surface.mode = dContactSoftERP | dContactSoftCFM;
			dc.surface.soft_erp = 0.1;
			dc.surface.soft_cfm = 0.00007;
			dc.surface.mu = dInfinity;
//			dc.surface.bounce = 0;
//			dc.surface.bounce_vel = 0.1;
			dc.geom = contact[i];

			dJointID cj = dJointCreateContact(wid, contactGroupid, &dc);
			dJointAttach(cj, b1, b2);
		}
	}
}

void step(dWorldID, double dt) {
	dSpaceCollide(sid, nullptr, &collisionCallback);
	dWorldStep(wid,dt);
	// Remove all joints in the contact joint group.
	dJointGroupEmpty(contactGroupid);
}
