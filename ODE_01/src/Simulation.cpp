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

Simulation::Simulation() {
	useBVH = false;
	// setup ODE
	initODE();
}

Simulation::Simulation(const char * bvhFile) {
	useBVH = true;
	simT = 0;

	// parse input file
	md.parse(bvhFile);

	// setup ODE
	initODE();
}

Simulation::~Simulation() {
	dWorldDestroy(wid);
}

void Simulation::step(double dt) {

	// load next keyframe according to time delta
	double simTcurrent = simT;
	simT += dt;
	int f = (int) (simT / md.getFrameTime());
	f = min(f, md.getNumFrames() - 1);
	loadFrame(f+3);

	// reflect changes in ODE
	// step ODE world in STEP_SIZE steps stopping just before current simT
	int odeSteps = (int) ((simT - odeSimT)/STEP_SIZE);
	odeSimT += odeSteps*STEP_SIZE;
	for(int i = 0; i < odeSteps; i++) {
		dSpaceCollide(sid, this, &collisionCallbackNonmemberFn);
		simTcurrent += STEP_SIZE;
		dWorldStep(wid,STEP_SIZE);
//		dWorldQuickStep(wid,STEP_SIZE);
		// Remove all joints in the contact joint group.
		dJointGroupEmpty(contactGroupid);

		// update joint angles to fit keyframe
		Vec3 pos;
		for(Skeleton * ss : getCurrentFrame()) {
//			// lock root node
//			dBodyID rootbid = skelBodyMap[ss->children[0]];
//			dBodySetKinematic(rootbid);
			for(Skeleton * s : ss->getAllSkeletons()) {
				dBodyID sbid = skelBodyMap[s->getLongName()];
				if(
						s->hasParent() &&
	//					(s->name == "LeftWrist" || s->name == "LeftElbow") &&
	//					(!s->parent->hasParent() || s->parent->parent->name == "Hips") &&
						sbid != 0) {
					Skeleton * p = s->parent;
					dBodyID pbid = skelBodyMap[p->getLongName()];
					if(pbid) {

						// use Slerp based torque
						Quat qi = eigQuat(dBodyGetQuaternion(sbid));
						Quat qf = eigQuat(dBodyGetQuaternion(pbid)) * p->getRot();

						// according to http://courses.cms.caltech.edu/cs171/quatut.pdf under 'Quaternion calculus' we can find an axis of rotation
						// this is the axis of our desired angular velocity
						Quat qlnTerm = qf * qi.inverse();
						const float angle = acos(qlnTerm.w());
						Vec3 rotAxis = qlnTerm.vec() / sin(angle);


						// use PD control
						const float kp = 5000;	// with killing angular velocity at each step, a good range is 1000 <= kp <= 6000
						const float kd = 10;

						const Quat ppjRotG_s = eigQuat(dBodyGetQuaternion(pbid));
						const Quat pjRotG_s  = eigQuat(dBodyGetQuaternion(sbid));
						const Quat pjRotL_s = ppjRotG_s.inverse() * pjRotG_s;
						const Quat pjRotL_k = p->getRot();
						const Vec3 posL			= pjRotL_s * s->getOffset();
						const Vec3 targetPosL	= pjRotL_k * s->getOffset();
//						const float errorP = acos((posL.dot(targetPosL)) / (posL.norm() * targetPosL.norm()));
						const float errorP = 2 * angle;// * acos((rotAxis.dot(posL)) / (rotAxis.norm() * posL.norm()));

						const float lastError = jointLastErrorMap[p];
						jointLastErrorMap[p] = errorP;
						const float errorD = (errorP - lastError) / STEP_SIZE;

						Vec3 torque = (
									(kp * errorP) +		// P
									(kd * errorD)		// D
								) * rotAxis;

//						// kill angular velocity
//						dBodySetAngularVel(sbid,0,0,0);
//						dBodySetAngularVel(pbid,0,0,0);
						// apply torque to joint
						float torqueNorm2 = torque.squaredNorm();
						if(!isnan(torqueNorm2) && !isinff(torqueNorm2) && torqueNorm2 != 0) {
							dBodyAddTorque(pbid, (dReal) -torque[0], (dReal) -torque[1], (dReal) -torque[2]);
							dBodyAddTorque(sbid, (dReal)  torque[0], (dReal)  torque[1], (dReal)  torque[2]);
						}

						// apply the force
	//					dBodyAddForce(sbid, (dReal) force[0], (dReal) force[1], (dReal) force[2]);



	//					// correct joint orientation relative to simulated parent joint (using keyframe as target)
	//					// not that sbid refers to the body with this joint's bone, but the parent joint's orientation/position
	//					// this is		ParentRotG_sim^-1 * RotG_sim	= 	RotL_sim	=   RotL_kf
	//					//				ParentRotG_sim^-1 * RotG_sim	=	RotL_kf
	//					//									RotG_sim	=	ParentRotG_sim * RotL_kf
	//					dBodySetQuaternion(sbid,q);
	//					// correct joint position which is
	//					//		Pos_sim = ParentPos_sim + ( ParentRotG_sim * Offset )
	//					dBodySetPosition(sbid, (dReal) target_pos[0], (dReal) target_pos[1], (dReal) target_pos[2]);
					}
				}
			}
//			dBodyID rootbid = skelBodyMap[ss->children[0]];
//			print(eigVec3(dBodyGetTorque(rootbid)));
		}
	}
}

void Simulation::initODE() {
	dInitODE();
	//	Create a dynamics world.
	wid = dWorldCreate();
	// set CFM
	dWorldSetCFM(wid,0.00007);
	dWorldSetERP(wid,0.1);
	//	Create a joint group to hold the contact joints.
	contactGroupid = dJointGroupCreate(1000);
	jointGroupid = dJointGroupCreate(100);
	/// gravity
//	dWorldSetGravity(wid, 0, -GRAVITY_ACC, 0);
	/// space
	sid = dHashSpaceCreate(0);
	/// floor
	dCreatePlane(sid, 0, 1, 0, -3);

	if(useBVH) {
		///////
		// read the first keyframe and realize the bones
		///////

		//	Set the state (position etc) of all bodies.
		for(Skeleton * ss : md.getKeyframe(3)) {
			initODESkeleton(ss, dBodyID());
		}

		// identify overlapping body segments
		dSpaceCollide(sid, this, &initialOverlapCollisionCallbackNonmemberFn);

	}

}

void Simulation::initODESkeleton(Skeleton* s, dBodyID parentBodyID) {

	// create a body for this bone
	dBodyID bid = dBodyCreate(wid);

	// create a geometry for bone and attach to parent body
	if (s->hasParent()) {
		Vec3 pos = s->getOffset();
		double height = pos.norm();

		// NOTE that capsules are aligned along the Z axis
		dGeomID bGeom = dCreateCapsule(sid, 3, height);
		dGeomSetBody(bGeom, parentBodyID);

		// set the local rotation and offset
		dQuaternion q;
		Quat qEigen = zToDirQuat(pos);
		dConv(qEigen, q);
		dGeomSetOffsetQuaternion(bGeom, q);
		dGeomSetOffsetPosition(bGeom, (dReal) (pos[0] * 0.5), (dReal) (pos[1] * 0.5), (dReal) (pos[2] * 0.5));

		// add mass to the parent
		dMass pMass;
		dBodyGetMass(parentBodyID, &pMass);
		pMass.mass += s->getMass();	// TODO have some better way of deciding mass
		dBodySetMass(parentBodyID, &pMass);
	}

	// set the position and orientation of the body
	Vec3 gPos = s->getPosG();
	dQuaternion gRot; dConv(s->getRotG(), gRot);
	dBodySetPosition(bid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);
	dBodySetQuaternion(bid, gRot);

	// attach a joint to parent
	if(s->hasParent()) {
		dJointID jid = dJointCreateBall(wid,jointGroupid);
		dJointAttach(jid,parentBodyID,bid);
		dJointSetBallAnchor(jid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);

//		// constrain join
//		dJointID amid = dJointCreateAMotor(wid,jointGroupid);
//		dJointAttach(amid,parentBodyID,bid);
//		dJointSetAMotorMode(amid,dAMotorEuler);
//		dJointSetAMotorAxis(amid,0,1, 0,0,1);
//		dJointSetAMotorAxis(amid,2,2, 0,-1,0);
//		dJointSetAMotorParam(amid,dParamLoStop,-PI/8);
//		dJointSetAMotorParam(amid,dParamHiStop,PI/8);

		// append to skelBodyMap
		skelBodyMap[s->getLongName()] = parentBodyID;
		bodySkelMap[parentBodyID] = s->getLongName();
	}

	// recurse to children
	for (vector<Skeleton*>::iterator c = s->children.begin();
			c != s->children.end(); c++) {
		initODESkeleton(*c, bid);
	}
}

