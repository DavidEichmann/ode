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
dBodyID bid;
dJointGroupID contactJointGroupID;
dSpaceID space;

Ogre::Root *mRoot;
Ogre::Camera* mCamera;
Ogre::SceneManager* mSceneMgr;
Ogre::RenderWindow* mWindow;
Ogre::SceneNode* sphereNode;


void collisionCallback(void *data, dGeomID o1, dGeomID o2) {

	// collect collision info
	const int maxC = 10;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, 0);

	// create collision joints
	for (int i = 0; i < c; i++) {
		dContact dc;
		dc.surface.mode = dContactBounce;
		dc.surface.mu = 1;
		dc.surface.bounce = 0.5;
		dJointID c = dJointCreateContact(wid, contactJointGroupID, &dc);
		dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
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
	dGeomID gid = dCreateSphere(space, 1);
	static dMass mass;
	dMassSetBox(&mass, 1, 1, 1, 1);
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
	mRoot = new Ogre::Root("", "", "");

	// Load the OpenGL RenderSystem and the Octree SceneManager plugins
	std::string pluginPath = "/usr/lib/i386-linux-gnu/OGRE-1.7.4/";
	mRoot->loadPlugin(pluginPath + "RenderSystem_GL");
	mRoot->loadPlugin(pluginPath + "Plugin_OctreeSceneManager");

	// configure the openGL / video settings
	Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
	if(!(rs->getName() == "OpenGL Rendering Subsystem"))
	{
	    return false; //No RenderSystem found
	}
	// configure our RenderSystem

	//////////////// AAAAAAAAAAAAAAAAAa

	rs->setConfigOption("Full Screen", "No");
	rs->setConfigOption("VSync", "No");
	std::string x = rs->getConfigOptions()["Video Mode"].currentValue;
	rs->setConfigOption("Video Mode", rs->getConfigOptions()["Video Mode"].currentValue);
	mRoot->setRenderSystem(rs);

	/// create generic scene manager
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
	/// create camera
	mCamera = mSceneMgr->createCamera("Cam");
	mCamera->setPosition(Ogre::Vector3(0, 0, 80));
	mCamera->lookAt(Ogre::Vector3(0, 5, 0));
	mCamera->setNearClipDistance(5);

	/// light
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	Ogre::Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(20,80,50);

	/// create window/viewport
	mWindow = mRoot->initialise(true, "My amazing simulation");
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(1, 1, 1));

	/// Alter the camera aspect ratio to match the viewport
	mCamera->setAspectRatio(
			Ogre::Real(vp->getActualWidth())
			/ Ogre::Real(vp->getActualHeight()));

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
		const float* pos = dBodyGetPosition(bid);
		sphereNode->setPosition(pos[0], pos[1], pos[2]);
		mRoot->renderOneFrame();
	}

	// destroy world
	destroyWorld();

	return 1;

}
