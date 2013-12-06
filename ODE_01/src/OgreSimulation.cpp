
#include <string.h>
#include <iostream>
#include <boost/chrono/system_clocks.hpp>
#include <time.h>
#include <ode/ode.h>
#include <OGRE/Ogre.h>
#include <OIS/OIS.h>

#include "Constants.h"
#include "Util.h"
#include "BVHParser.h"
#include "Procedural.h"
#include "OgreSimulation.h"

OgreSimulation::OgreSimulation(const char * bvhFile) : Simulation(bvhFile) {};

void OgreSimulation::run() {
	initOgre();
	realizeSkeletons();
	mainLoop();
}

void OgreSimulation::initOgre() {
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
	l->setPosition(200, 200, 50);
}

void OgreSimulation::mainLoop() {
	// setup timing variables
	boost::chrono::system_clock::time_point it =
			boost::chrono::system_clock::now(); 	// initial time
	double dt;
	// main loop
	while (!mWindow->isClosed()) {
		// get the current frame
		dt = ((boost::chrono::nanoseconds) (boost::chrono::system_clock::now()
				- it)).count() / ((double) 1000000000);
		it = boost::chrono::system_clock::now();

		// step the world
		step(dt);

		// rotate the camera
		double h = 100;
		double d = 400;
		double s = 10;
		mCamera->setPosition(
				Ogre::Vector3(d * Ogre::Math::Sin(dt / s), h,
						d * Ogre::Math::Cos(dt / s)));
		mCamera->lookAt(Ogre::Vector3(0, h, 0));

		// update the position of all bones
		updateFromSim();

		// Render the world
		Ogre::WindowEventUtilities::messagePump();
		mRoot->renderOneFrame(10);
	}
}

void OgreSimulation::realizeSkeletons() {
	nodeSkelPairs.clear();
	for(vector<Skeleton*>::iterator i = bvh.skeletons.begin(); i != bvh.skeletons.end(); i++) {
		realizeSkeleton(*i);
	}
}

void OgreSimulation::realizeSkeleton(Skeleton * s) {
	// ignore root and 0 length nodes
	Vec3 pos = s->getPos();
	double height = pos.norm();
	if(s->hasParent() && height > 0) {

		// create a global scene node
		Ogre::SceneNode * node = mSceneMgr->getRootSceneNode()->createChildSceneNode();

		// generate a mesh
		Procedural::CapsuleGenerator gen = Procedural::CapsuleGenerator();
		gen.setRadius(3);

		/// height
		gen.setHeight(height);
		//gen.setHeight(1);

		//  Achieve this...
		//    orientation: s's local position is a vector representing the orientation of the bone
		//    position: one end at the origin, the other ending at (-1 * local position) a.k.a heading back to parent node
		//  ...by positioning the bone on the positive y axis, then rotating to correct orientation
		//  handle the special case of pos being on the y axis (cross product will fail)
		if(pos[0] == 0 && pos[2] == 0) {
			if(pos[1] > 0) {
				gen.setPosition(toVec3(pos * -0.5));
			}
			else {
				gen.setPosition(toVec3(pos * 0.5));
			}
		}
		else {
			Vec3 dirU = (pos * (-1)).normalized();
			Vec3 initU = Vec3::UnitY();
			Vec3 rotAxis = initU.cross(dirU).normalized();
			double rotAngle = acos(initU.dot(dirU));
			Quaterniond meshRot = (Quaterniond) AngleAxisd(rotAngle, rotAxis);

			const Ogre::Vector3 finalPos = toVec3( (Vec3) (
					// rotate around rotX to get from initU to posU
					meshRot *
					// rotate the position at the half length in the irection of initU
					(initU * (height/2))) );
			gen.setPosition(finalPos*-1);
			gen.setOrientation(toQuat(meshRot));
		}

		Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
		se->setMaterialName("Examples/BeachStones");
		node->attachObject(se);

		// save node/skeleton pair
		nodeSkelPairs.push_back(pair<Ogre::SceneNode*,Skeleton*>(node,s));
	}

	// recurse to children
	for(vector<Skeleton*>::iterator i = s->children.begin(); i != s->children.end(); i++) {
		realizeSkeleton(*i);
	}
}

void OgreSimulation::updateFromSim() {
	for(vector< pair<Ogre::SceneNode*,Skeleton*> >::iterator it = nodeSkelPairs.begin(); it != nodeSkelPairs.end(); it++) {
		Ogre::SceneNode * sn = it->first;
		Skeleton * sk = it->second;

		// ensure that there is a valid dBodyID
		if(skelBodyMap.count(sk) != 0) {
			dBodyID bodyID = skelBodyMap[sk];
			const double * pos = dBodyGetPosition(bodyID);
			const double * q = dBodyGetQuaternion(bodyID);

			sn->setPosition(Ogre::Vector3(pos[0],pos[1],pos[2]));
			sn->setOrientation(Ogre::Quaternion(q[0],q[1],q[2],q[3]));
		}
	}
}


