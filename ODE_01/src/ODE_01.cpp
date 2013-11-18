//============================================================================
// Name        : ODE_01.cpp
// Author      : David Eichmann
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <string.h>
#include <iostream>
#include <ode/ode.h>
#include <OGRE/Ogre.h>
#include <OIS/OIS.h>

#include "Constants.h"



dWorldID wid;
static dBodyID bid;
dJointGroupID contactJointGroupID;
dSpaceID space;

Ogre::Root *mRoot;
Ogre::Camera* mCamera;
Ogre::SceneManager* mSceneMgr;
Ogre::RenderWindow* mWindow;
Ogre::SceneNode* sphereNode;


void collisionCallback(void *data, dGeomID o1, dGeomID o2) {
	// ignore collisions with spaces
	if(dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		return;
	}

	// collect collision info
	const int maxC = 10;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, (int)sizeof(dContactGeom));

	// create collision joints
	for (int i = 0; i < c; i++) {
		dContact dc;

		dc.surface.mode = dContactBounce;
		dc.surface.mu = 1;
		dc.surface.bounce = 0.5;
		dc.surface.bounce_vel = 0.1;

		dJointID c = dJointCreateContact(wid, contactJointGroupID, &dc);
		dBodyID b1 = dGeomGetBody(o1);
		dBodyID b2 = dGeomGetBody(o2);
		dJointAttach(c, b1, b2);
	}
}

void stepWorld() {

	//		Apply forces to the bodies as necessary.
	//		Adjust the joint parameters as necessary.
	//		Call collision detection.
	dSpaceCollide(space, NULL, collisionCallback);
	//		Create a contact joint for every collision point, and put it in the contact joint group.
	//		Take a simulation step.
	dWorldStep(wid, STEP_SIZE);
	//		Remove all joints in the contact joint group.

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
	float fd = mass.mass;
	dMassSetSphere(&mass, 1, 1);
	fd = mass.mass;
	dBodySetMass(bid, &mass);
	dGeomSetBody(gid, bid);

	//	Set the state (position etc) of all bodies.
	dBodySetPosition(bid, 0, 10, 0);

	//	Create joints in the dynamics world.

	//	Attach the joints to the bodies.

	//	Set the parameters of all joints.

	//	Create a collision world and collision geometry objects, as necessary.
	// ??? already done ???

	//	Create a joint group to hold the contact joints.
	contactJointGroupID = dJointGroupCreate(1000);
}

int main(int pargc, char** argv) {

	// init ogre
	/// Create root
	mRoot = new Ogre::Root("", "ogre.cfg", "ogre.log");

	// Load the OpenGL RenderSystem and the Octree SceneManager plugins
	std::string pluginPath = "/usr/lib/i386-linux-gnu/OGRE-1.8.0/";
	mRoot->loadPlugin(pluginPath + "RenderSystem_GL");
	mRoot->loadPlugin(pluginPath + "Plugin_BSPSceneManager");
	mRoot->loadPlugin(pluginPath + "Plugin_PCZSceneManager");
	mRoot->loadPlugin(pluginPath + "Plugin_OctreeZone");
	mRoot->loadPlugin(pluginPath + "Plugin_OctreeSceneManager");
	// mRoot->loadPlugin(pluginPath + "Plugin_ParticleFX0");

	mRoot->addResourceLocation("../Media", "FileSystem", "Group", true);

	mRoot->restoreConfig() || mRoot->showConfigDialog();
//	// configure the openGL / video settings
//	Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
//	if(!(rs->getName() == "OpenGL Rendering Subsystem"))
//	{
//	    return false; //No RenderSystem found
//	}
//
//
//	rs->setConfigOption("Full Screen", "No");
//	rs->setConfigOption("VSync", "No");
//	rs->setConfigOption("Video Mode", "640 x  480");
//	rs->setConfigOption("sRGB Gamma Conversion", "No");
//	rs->setConfigOption("RTT Preferred Mode", "FBO");
//	rs->setConfigOption("FSAA", "No");
//	rs->setConfigOption("Display Frequency", "58 MHz");
//	mRoot->setRenderSystem(rs);

	// initialize ogre
	mWindow = mRoot->initialise(true, "My amazing simulation");

	/// create generic scene manager
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	/// create camera
	mCamera = mSceneMgr->createCamera("My Cam");
	mCamera->setPosition(Ogre::Vector3(0, 0, 1000));
	mCamera->lookAt(Ogre::Vector3(0, 0, 0));
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
	l->setPosition(20,80,50);

	/// Create sphere
	Ogre::Entity* se = mSceneMgr->createEntity("My Sphere", Ogre::SceneManager::PT_SPHERE);
	sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("My Sphere Node");
	sphereNode->attachObject(se);


	// world contents
	dInitODE();
	initWorld();

	// enter GLUT event processing cycle
	while( ! mWindow->isClosed()) {
		stepWorld();
		const dReal* pos = dBodyGetPosition(bid);
		dReal scale = 0.5;
		sphereNode->setPosition(scale*pos[0], scale*pos[1], scale*pos[2]);
		std::cout << pos[1] << std::endl;
		Ogre::WindowEventUtilities::messagePump();
		mRoot->renderOneFrame(10);
	}

	// destroy world
	destroyWorld();

	return 1;

}
