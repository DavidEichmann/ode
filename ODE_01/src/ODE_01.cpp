//============================================================================
// Name        : ODE_01.cpp
// Author      : David Eichmann
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <string.h>
#include <iostream>
#include <boost/chrono/system_clocks.hpp>
#include <time.h>
#include <ode/ode.h>
#include <OGRE/Ogre.h>
#include <OIS/OIS.h>

#include "Constants.h"
#include "BVHParser.h"
#include "Procedural.h"
#include "ODE_01.h"

dWorldID wid;
dJointGroupID contactJointGroupID;


void collisionCallback(void *data, dGeomID o1, dGeomID o2) {

	// ignore collisions with spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		return;
	}

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	// collect collision info
	const int maxC = 10;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));

	// create collision joints
	for (int i = 0; i < c; i++) {

		dContact dc;

		dc.surface.mode = dContactBounce;
		dc.surface.mu = 1;
		dc.surface.bounce = 0.5;
		dc.surface.bounce_vel = 0.1;
		dc.geom = contact[i];

		dJointID cj = dJointCreateContact(wid, contactJointGroupID, &dc);
		dJointAttach(cj, b1, b2);
	}
}


class Main {

public:

	static dBodyID bid;
	dSpaceID space;

	Ogre::Root *mRoot;
	Ogre::Camera* mCamera;
	Ogre::SceneManager* mSceneMgr;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneNode* sphereNode;

	BVHParser * bvh;


	/**
	 * dt: target simulation time
	 * returns the time actually simulated
	 */
	double stepWorld(double dt) {

		//		Take simulation step (consists of potentially many ode world steps).
		double timeSim = 0;
		while (timeSim < dt) {
			//		Apply forces to the bodies as necessary.
			//		Adjust the joint parameters as necessary.
			//		Call collision detection.
			dSpaceCollide(space, NULL, &collisionCallback);
			//		Create a contact joint for every collision point, and put it in the contact joint group.
			dWorldStep(wid, STEP_SIZE);
			timeSim += STEP_SIZE;

			//		Remove all joints in the contact joint group.
			dJointGroupEmpty(contactJointGroupID);
		}

		return timeSim;

	}

	void destroyWorld() {

		//	Destroy the dynamics and collision worlds.
		dWorldDestroy(wid);

	}

	void initWorld() {

		//	Create a dynamics world.
		wid = dWorldCreate();
		/// gravity
		dWorldSetGravity(wid, 0, GRAVITY_ACC, 0);
		/// space
		space = dHashSpaceCreate(0);
		/// floor
		dCreatePlane(space, 0, 1, 0, 0);

		//	Create bodies in the dynamics world.
		/// simple sphere
		bid = dBodyCreate(wid);
		static dGeomID gid = dCreateSphere(space, 1);
		static dMass mass;
		dMassSetSphere(&mass, 1, 1);
		dBodySetMass(bid, &mass);
		dGeomSetBody(gid, bid);

		//	Set the state (position etc) of all bodies.
		dBodySetPosition(bid, 0, 50, 0);

		//	Create joints in the dynamics world.

		//	Attach the joints to the bodies.

		//	Set the parameters of all joints.

		//	Create a collision world and collision geometry objects, as necessary.
		// ??? already done ???

		//	Create a joint group to hold the contact joints.
		contactJointGroupID = dJointGroupCreate(1000);
	}

	void realizeSkeleton(Skeleton * s) {

		// generate node and mesh and attach to parent node else root
		Ogre::MeshPtr seM;
		Ogre::SceneNode* node;
		if(s->hasParent() && s->parent->hasOgreNode()) {
			node = s->parent->ogreNode->createChildSceneNode();
		}
		else {
			node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		}
		const double * xyz = s->getPosXYZ();
		node->setPosition(xyz[0],xyz[1],xyz[2]);	// this is the OFFSET from the bvh file
		/// Create sphere if root joint or height 0 bone
		Ogre::Vector3 pos = Ogre::Vector3(xyz[0],xyz[1],xyz[2]);
		double height = pos.length();
		if(! s->hasParent() || height == 0) {
			seM = Procedural::SphereGenerator().setRadius(2).realizeMesh();
			Ogre::Entity* se = mSceneMgr->createEntity(seM);
			node->attachObject(se);
		}
		// else create a capsle bone attached to parent's Scene node
		else {
			Procedural::CapsuleGenerator gen = Procedural::CapsuleGenerator();
			gen.setRadius(1);
			gen.setHeight(height);
			//Ogre::Quaternion rotZ = Ogre::Quaternion(Ogre::Vector3::UNIT_Y.angleBetween(pos), Ogre::Vector3::UNIT_Z);
			//Ogre::Quaternion rotY = Ogre::Quaternion(Ogre::Math::ATan2(xyz[0],xyz[2]), Ogre::Vector3::UNIT_Y);
			//gen.setOrientation(rotZ * rotY);
			Ogre::Radian a = Ogre::Vector3::UNIT_Y.angleBetween(pos);
			Ogre::Vector3 axis = Ogre::Vector3::UNIT_Y.crossProduct(pos).normalisedCopy();
			Ogre::Quaternion rot = Ogre::Quaternion(a, axis);
			if(axis.isZeroLength()) {
				if(a == Ogre::Radian(0)) {
					rot = Ogre::Quaternion(Ogre::Radian(0), Ogre::Vector3::UNIT_X);
				}
				else {
					rot = Ogre::Quaternion(a, Ogre::Vector3::UNIT_X);
				}
			}

			gen.setPosition(rot * Ogre::Vector3(0,height/2,0));
			gen.setOrientation(rot);
//			// orientate down -Z so that the SceneNode lookAt function will take care
//			// of updating orientation
//			gen.setOrientation(Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI/2), Ogre::Vector3::UNIT_X));
			seM = gen.realizeMesh();
			Ogre::Entity* se = mSceneMgr->createEntity(seM);
			s->parent->ogreNode->attachObject(se);
		}
		s->ogreNode = node;

		// recurse to children
		realizeSkeletons(& s->children);

	}

	void realizeSkeletons(vector<Skeleton*> * ss = NULL) {
		if(ss == NULL) { ss = & bvh->skeletons; }
		for(vector<Skeleton*>::iterator it = (*ss).begin(); it != (*ss).end(); it++) {
			realizeSkeleton(*it);
		}
		if(ss == & bvh->skeletons) {
			updateSkeletons();
		}
	}

	void updateSkeleton(Skeleton * s) {
		// update pos and rot
		const double * posXYZ;
		posXYZ = s->getPosXYZ();
		s->ogreNode->setPosition(posXYZ[0], posXYZ[1], posXYZ[2]);
		const double * xyz = s->getRotXYZ();
		Ogre::Quaternion qX = Ogre::Quaternion(Ogre::Radian(Ogre::Math::DegreesToRadians(xyz[0])),Ogre::Vector3::UNIT_X);
		Ogre::Quaternion qY = Ogre::Quaternion(Ogre::Radian(Ogre::Math::DegreesToRadians(xyz[1])),Ogre::Vector3::UNIT_Y);
		Ogre::Quaternion qZ = Ogre::Quaternion(Ogre::Radian(Ogre::Math::DegreesToRadians(xyz[2])),Ogre::Vector3::UNIT_Z);
		// apply rotations in BVH order: Y then X then Z
		Ogre::Quaternion q = qZ * qX * qY;
		//Ogre::Quaternion q = qZ;
		s->ogreNode->setOrientation(q);

		// recurse to children
		for (vector<Skeleton*>::iterator it = s->children.begin(); it != s->children.end(); it++) {
			updateSkeleton(*it);
		}
	}

	void updateSkeletons() {
		for (vector<Skeleton*>::iterator it = bvh->skeletons.begin(); it != bvh->skeletons.end(); it++) {
			updateSkeleton(*it);
		}
	}

	int run() {

		//bvh = new BVHParser("Data/yoga_gym_yoga3_1_c3d.bvh");
		bvh = new BVHParser("Data/b_Boxer.shadow17_1_s.bvh");
		//bvh = new BVHParser("Data/mgman__3cut4_2_x2d.bvh");

		// init ogre
		/// Create root
		mRoot = new Ogre::Root("", "ogre.cfg", "ogre.log");

		// Load the OpenGL RenderSystem and the Octree SceneManager plugins
		std::string pluginPath = "/usr/lib/x86_64-linux-gnu/OGRE-1.8.0/";
		mRoot->loadPlugin(pluginPath + "RenderSystem_GL");
		mRoot->loadPlugin(pluginPath + "Plugin_BSPSceneManager");
		mRoot->loadPlugin(pluginPath + "Plugin_PCZSceneManager");
		mRoot->loadPlugin(pluginPath + "Plugin_OctreeZone");
		mRoot->loadPlugin(pluginPath + "Plugin_OctreeSceneManager");
		// mRoot->loadPlugin(pluginPath + "Plugin_ParticleFX0");

		mRoot->addResourceLocation("../Media", "FileSystem", "Group", true);

		mRoot->restoreConfig() || mRoot->showConfigDialog();

		// initialize ogre
		mWindow = mRoot->initialise(true, "My amazing simulation");

		/// create generic scene manager
		mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

		/// create camera
		mCamera = mSceneMgr->createCamera("My Cam");
		mCamera->setPosition(Ogre::Vector3(300, 500, 300));
		mCamera->lookAt(Ogre::Vector3(0, 100, 0));
		mCamera->setNearClipDistance(5);
		mCamera->setFarClipDistance(10000);

		/// create window/viewport
		Ogre::Viewport* vp = mWindow->addViewport(mCamera);
		vp->setBackgroundColour(Ogre::ColourValue(0.5, 0.5, 1));
		// Alter the camera aspect ratio to match the viewport
		mCamera->setAspectRatio(
				Ogre::Real(vp->getActualWidth())
						/ Ogre::Real(vp->getActualHeight()));

		Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

		/// light
		mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
		Ogre::Light* l = mSceneMgr->createLight("MainLight");
		l->setPosition(20, 80, 50);





		// FOR NOW JUST RENDER ANNIMATION
		realizeSkeletons(); // realize the parsed skeletons

		// setup timing variables
		boost::chrono::system_clock::time_point it = boost::chrono::system_clock::now(); 	// initial time
		double dt;
		int f;
		// main loop
		while(!mWindow->isClosed()) {
			// get the current frame
			dt = ((boost::chrono::nanoseconds) (boost::chrono::system_clock::now() - it)).count() / ((double) 1000000000);

			// step the world
			double speed = 1;
			f = (int) ((dt / bvh->frameTime) * speed);
			if(f >= bvh->numFrames) { break; }
			bvh->loadKeyframe(f);
			//bvh->loadKeyframe(0);
			double h = 100;
			double d = 300;
			double s = 1000;
			mCamera->setPosition(Ogre::Vector3(d*Ogre::Math::Sin(dt/s), h, d*Ogre::Math::Cos(dt/s)));
			mCamera->lookAt(Ogre::Vector3(0, h, 0));


			// update the position of all bones
			updateSkeletons();

			// Render the world
			Ogre::WindowEventUtilities::messagePump();
			mRoot->renderOneFrame(10);
		}

		// destroy world
		destroyWorld();

		return 0;

	/*





		/// Create sphere
		string name = "sphere";
		Ogre::MeshPtr seM = Procedural::SphereGenerator().setRadius(10).realizeMesh(name);
		Ogre::Entity* se = mSceneMgr->createEntity(seM);
		se->setMaterialName("Examples/OgreLogo");
		sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(
				"My Sphere Node");
		sphereNode->attachObject(se);

		// world contents
		dInitODE();
		initWorld();

		// setup timing variables
		boost::chrono::system_clock::time_point it,ft; 	// initial time, final time (between frames)
		ft = boost::chrono::system_clock::now();
		double dt;		// time delta of the frame
		double simTOffset = 0;	// real time (seconds) - time in the simulation
		// mian loop
		while (!mWindow->isClosed()) {
			// get the time and time delta
			it = ft;
			ft = boost::chrono::system_clock::now();
			dt = ((boost::chrono::nanoseconds) (ft - it)).count() / ((double) 1000000000);
			simTOffset += dt;


			// step the world
			simTOffset -= stepWorld(simTOffset);

			// update the position of the sphere (from ODE world to OGRE world)
			const dReal* pos = dBodyGetPosition(bid);
			dReal scale = 7;
			sphereNode->setPosition(scale * pos[0], scale * pos[1], scale * pos[2]);

			// Render the world
			Ogre::WindowEventUtilities::messagePump();
			mRoot->renderOneFrame(10);
		}

		// destroy world
		destroyWorld();

		return 0;
	//*/

	}
};



int main(int pargc, char** argv) {
	Main m = Main();
	return m.run();
}
