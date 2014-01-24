#include "Simulation.h"

#include <string.h>
#include <btBulletCollisionCommon.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "All.h"





void initialOverlapCollisionCallbackNonmemberFn(void * sim, dGeomID o1, dGeomID o2) {
//	((Simulation *) sim)->initialOverlapCallback(o1, o2);
}

void collisionCallbackNonmemberFn(void * sim, dGeomID o1, dGeomID o2) {
//	((Simulation *) sim)->collisionCallback(o1, o2);
}

void Simulation::initialOverlapCallback(dGeomID o1, dGeomID o2) {
//	dBodyID b1 = dGeomGetBody(o1);
//	dBodyID b2 = dGeomGetBody(o2);
//
//	// only look for bone on bone collisions
//	if(bodySkelMap.count(b1) == 0 || bodySkelMap.count(b2) == 0 ) {
//		return;
//	}
//
//	// collect collision info
//	const int maxC = 1;
//	dContactGeom contact[maxC];
//	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));
//
//	if(c > 0) {
//		setOverlap(b1,b2);
//	}
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

//void Simulation::collisionCallback(dGeomID o1, dGeomID o2) {
//
//	// ignore collisions with spaces
//	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
//		return;
//	}
//
//	dBodyID b1 = dGeomGetBody(o1);
//	dBodyID b2 = dGeomGetBody(o2);
//
//	// don't collide overlapping/connected bones
//	if(b1 != 0 && b2 != 0 && (dAreConnectedExcluding(b1,b2,dJointTypeContact) || overlap(b1,b2))) {
//		return;
//	}
//
////	// only plane collisions
////	if(dGeomGetClass(o1) != dPlaneClass && dGeomGetClass(o2) != dPlaneClass) {
////		return;
////	}
//
//	// collect collision info
//	const int maxC = 10;
//	dContactGeom contact[maxC];
//	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));
//
////	if(c > 0 && dGeomGetClass(o1) != dPlaneClass && dGeomGetClass(o2) != dPlaneClass) {
////		cout << "collision between: " << bodySkelMap[b1]->name << " and " << bodySkelMap[b2]->name << endl;
////		cout << contact[0].pos[0] << ", " << contact[0].pos[1] << ", " << contact[0].pos[2] << endl;
////	}
//
//	// create collision joints
//	for (int i = 0; i < c; i++) {
//		if(contact[i].depth != 0) {
//			dContact dc;
//
//			dc.surface.mode = dContactSoftERP;
//			dc.surface.soft_erp = 0.1;
//			dc.surface.mu = 1;
////			dc.surface.bounce = 0;
////			dc.surface.bounce_vel = 0.1;
//			dc.geom = contact[i];
//
//			dJointID cj = dJointCreateContact(wid, contactGroupid, &dc);
//			dJointAttach(cj, b1, b2);
//		}
//	}
//}

Simulation::Simulation() {
	useBVH = false;
	// setup ODE
	initODE();
}

Simulation::Simulation(const char * bvhFile) {
	useBVH = true;
	simT = 0;

	// parse input file
	bvh.parse(bvhFile);

	// setup ODE
	initODE();
}

void Simulation::step(double dt) {

	// load next keyframe according to time delta
	double simTcurrent = simT;
	simT += dt;
	int f = (int) (simT / bvh.frameTime);
	f = min(f, bvh.numFrames - 1);
	//bvh.loadKeyframe(f);

	// reflect changes in Bullet
	// step Bullet world in STEP_SIZE steps stopping just before current simT
	int odeSteps = (int) ((simT - odeSimT)/STEP_SIZE);
	odeSimT += odeSteps*STEP_SIZE;
	for(int i = 0; i < odeSteps; i++) {
		simTcurrent += STEP_SIZE;
		world->stepSimulation(STEP_SIZE,1,STEP_SIZE);
//		cout << skelBodyMap[bvh.skeletons[0]->children[0]]->getWorldTransform().getOrigin().y() << endl;
	}
}

vector<Skeleton*> Simulation::getSkeletons() {
	return bvh.skeletons;
}

void Simulation::initODE() {

	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	world->setGravity(btVector3(0,-GRAVITY_ACC,0));

	// floor
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
	btRigidBody* groundRigidBody = new btRigidBody(0, NULL, groundShape, btVector3(0,0,0));
	world->addRigidBody(groundRigidBody);

	if(useBVH) {
		///////
		// read the first keyframe and realize the bones
		///////
		bvh.loadKeyframe(0);

		//	Set the state (position etc) of all bodies.
		for (vector<Skeleton*>::iterator ss = bvh.skeletons.begin();
				ss != bvh.skeletons.end(); ss++) {
			initODESkeleton(*ss, nullptr);
		}

		// TODO identify overlapping body segments

	}

}

void Simulation::initODESkeleton(Skeleton* s, btRigidBody* parent) {

	// create a geometry for bone and attach to parent body
	btCompoundShape* boneShape = new btCompoundShape();
	btRigidBody* body = new btRigidBody(1, NULL, boneShape, btVector3(0,0,0));
	if (s->hasParent()) {
		Vec3 pos = s->getOffset();
		double height = pos.norm();

		// NOTE that capsules are aligned along the Z axis
		btVector3 btVec;
		btQuaternion btQuat;
		if (pos[0] == 0 && pos[1] == 0) {
			btVec = btVector3(0, 0, pos[2] * 0.5);
			btQuat = btQuat.getIdentity();
		}
		else {
			Vec3 iDir = Vec3::UnitZ();
			Vec3 tDir = pos.normalized();
			Vec3 axis = iDir.cross(tDir).normalized();
			double angle = acos(iDir.dot(tDir));
			//dQFromAxisAndAngle(q,(dReal)axis[0],(dReal)axis[1],(dReal)axis[2],(dReal)angle);
			Quat qEigen = (Quaterniond) AngleAxisd(angle, axis);
			Vec3 fPos = tDir * (height/2);
			btVec = btConv(fPos);
			btQuat = btConv(qEigen);
		}
		((btCompoundShape*) parent->getCollisionShape())->addChildShape(btTransform(btQuat, btVec), new btCapsuleShapeZ(0.1, height));
		// add mass to the parent
		btScalar mass = 1 / parent->getInvMass();
		mass += height * 20;	// TODO have some better way of deciding mass
		btVector3 inertia;
		parent->getCollisionShape()->calculateLocalInertia(mass,inertia);
		parent->setMassProps(mass, inertia);
	}

	// set the position and orientation of the body
	body->setWorldTransform(btTransform(btConv(s->getRotG()), btConv(s->getPosEndG())));

	// TODO attach a joint to parent
	if(s->hasParent()) {
//		dJointID jid = dJointCreateBall(wid,jointGroupid);
//		dJointAttach(jid,parentBodyID,bid);
//		dJointSetBallAnchor(jid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);
//
//		// constrain join
//		dJointID amid = dJointCreateAMotor(wid,jointGroupid);
//		dJointAttach(amid,parentBodyID,bid);
//		dJointSetAMotorMode(amid,dAMotorEuler);
//		dJointSetAMotorAxis(amid,0,1, 0,0,1);
//		dJointSetAMotorAxis(amid,2,2, 0,-1,0);
//		dJointSetAMotorParam(amid,dParamLoStop,-PI/8);
//		dJointSetAMotorParam(amid,dParamHiStop,PI/8);

		world->addConstraint(new btPoint2PointConstraint(*parent, *body, btConv(s->getOffset()), btVector3(0,0,0)), true);

		// append to skelBodyMap
		skelBodyMap[s] = parent;
		bodySkelMap[parent] = s;
	}

	// recurse to children
	for (vector<Skeleton*>::iterator c = s->children.begin();
			c != s->children.end(); c++) {
		initODESkeleton(*c, body);
	}

	// add to the world
	world->addRigidBody(body);
}
