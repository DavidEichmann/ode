
#include <string.h>
#include <iostream>
#include <boost/chrono/system_clocks.hpp>
#include <time.h>
#include <ode/ode.h>
#include <OGRE/Ogre.h>
#include <OIS/OIS.h>
#include <Eigen/Geometry>
#include <tuple>

#include "Simulation.h"
#include "Constants.h"
#include "Util.h"
#include "MotionData.h"
#include "Procedural.h"
#include "OgreSimulation.h"

OgreSimulation::OgreSimulation() : Simulation() {};
OgreSimulation::OgreSimulation(const char * bvhFile) : Simulation(bvhFile) {};

void OgreSimulation::run() {
	initOgre();
	// realizeHuman();
	// realizeBall();
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
	mCamera->setNearClipDistance(0.05);
	mCamera->setFarClipDistance(100);

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
	boost::chrono::system_clock::time_point startT =
			boost::chrono::system_clock::now(); 	// initial time
	double dt, t;
	// main loop
	while (!mWindow->isClosed()) {
		// get the current frame
		t = ((boost::chrono::nanoseconds) (boost::chrono::system_clock::now()
				- startT)).count() / ((double) 1000000000);
		dt = ((boost::chrono::nanoseconds) (boost::chrono::system_clock::now()
				- it)).count() / ((double) 1000000000);
		it = boost::chrono::system_clock::now();

		// step the world
		step(dt);

		// rotate the camera
		double h = 1;
		double d = 7;
		double s = -20;
		mCamera->setPosition(
				Ogre::Vector3(d * Ogre::Math::Sin(t / s), h,
						d * Ogre::Math::Cos(t / s)));
		mCamera->lookAt(Ogre::Vector3(0, h, 0));

		// draw the scene
		draw();
		doRender();
	}
}

void OgreSimulation::realizeHuman() {
	human = new Human("Data/Model/Human_v0.2.model");
	human->realize(wid,sid);
	int size = human->getSize();
	humanNodes = new Ogre::SceneNode*[size];
	const Human::Props * props = human->getProps();
	for(int i = 0; i < size; i++) {

		// create a global scene node
		Ogre::SceneNode * node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		humanNodes[i] = node;

		//position
		Vec3 pos = props->pos;
		node->setPosition((float) pos[0], (float) pos[1], (float) pos[2]);

		// generate a mesh
		Procedural::BoxGenerator gen = Procedural::BoxGenerator();
		gen.setSizeX(props->width);
		gen.setSizeY(props->height);
		gen.setSizeZ(props->depth);

		// attach to node
		Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
		se->setMaterialName("Ogre/Earring");
		node->attachObject(se);

		props++;
	}
}

void OgreSimulation::draw() {

	if(human != NULL) {
		int size = human->getSize();
		const dBodyID * bID = human->getBodyIDs();
		Ogre::SceneNode * * sn = humanNodes;
		for(int i = 0; i < size; i++) {
			const dReal * pos = dBodyGetPosition(*bID);
			const dReal * q = dBodyGetQuaternion(*bID);
			(*sn)->setPosition(Ogre::Vector3(pos[0],pos[1],pos[2]));
			(*sn)->setOrientation(Ogre::Quaternion(q[0],q[1],q[2],q[3]));
			bID++;
			sn++;
		}
	}
	else {
		for(Skeleton * ss : getCurrentFrame()) {
			for(Skeleton * s : ss->getAllSkeletons()) {
				if(s->hasParent()) {
					drawBone(s->getPosStartG(), s->getPosEndG());
				}
			}
			drawPoint(ss->getZMP());
			Vec3 comp = ss->getPosTotalComG();
			comp[1] = 0;
			drawPoint(comp);
		}
	}
//	else {
//		vector<vector<Skeleton*> > skelsAll;
//		for(Skeleton * s : getCurrentFrame()) {
//			skelsAll.push_back(s->getAllSkeletons());
//		}
//
//		for(pair<Ogre::SceneNode*,Vector2i> & pair : nodeSkelIndexPairs) {
//			Ogre::SceneNode * sn = pair.first;
//			Skeleton * sk = skelsAll[ pair.second[0] ][ pair.second[1] ];
//
//			// ensure that there is a valid dBodyID
//			if(skelBodyMap.count(sk->getLongName()) != 0) {
//				dBodyID bodyID = skelBodyMap[sk->getLongName()];
//
//				const dReal * pos = dBodyGetPosition(bodyID);
//				const dReal * q = dBodyGetQuaternion(bodyID);
//				sn->setPosition(Ogre::Vector3(pos[0],pos[1],pos[2]));
//				sn->setOrientation(Ogre::Quaternion(q[0],q[1],q[2],q[3]));
//
//			}
//
//			// animation
//			// ensure that there is a parent joint
//			if(sk->hasParent()) {
//				Ogre::SceneNode * aniSn = nodeAniNodeMap[sn];
//				aniSn->setPosition(ogreConv(sk->parent->getPosG() + Vec3(-3.5,0,0)));
//				aniSn->setOrientation(ogreConv(sk->parent->getRotG()));
//			}
//		}
//	}
}

void OgreSimulation::draw(Ogre::Entity * e, Vec3 pos, Quat rot) {
	Ogre::SceneNode * node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	node->setPosition(ogreConv(pos));
	node->setOrientation(ogreConv(rot));
	node->attachObject(e);
}

void OgreSimulation::doRender() {
	// Render the world
	Ogre::WindowEventUtilities::messagePump();
	mRoot->renderOneFrame(10);

	// remove all nodes for next frame
	mSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
}

void OgreSimulation::drawBone(Vec3 start, Vec3 end, double radius) {
	drawVec3(start, end-start, radius);
};

void OgreSimulation::drawVec3(Vec3 origin, Vec3 vec, double radius) {
				// generate a mesh
				Procedural::CapsuleGenerator gen = Procedural::CapsuleGenerator();
				gen.setRadius(radius);
				gen.setHeight(vec.norm());

				// create an entity
				Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
				se->setMaterialName("Ogre/Earring");
				draw(se, origin + (vec * 0.5), yToDirQuat(vec));
}

void OgreSimulation::drawPoint(Vec3 p, double radius) {
				// generate a mesh
				Procedural::SphereGenerator gen = Procedural::SphereGenerator();
				gen.setRadius(radius);

				// create an entity
				Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
				se->setMaterialName("Ogre/Earring");
				draw(se, p);
}
