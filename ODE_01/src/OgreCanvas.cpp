
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
#include "OgreCanvas.h"

OgreCanvas::OgreCanvas() {};

void OgreCanvas::initOgre() {
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
	mCamera->setPosition(Ogre::Vector3(0, 0, 7));
	mCamera->lookAt(Ogre::Vector3(0, 0, 0));
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


void OgreCanvas::draw(Ogre::Entity * e, Vec3 pos, Quat rot) {
	Ogre::SceneNode * node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	node->setPosition(ogreConv(pos));
	node->setOrientation(ogreConv(rot));
	node->attachObject(e);
}

bool OgreCanvas::doRender() {
	// Render the world
	Ogre::WindowEventUtilities::messagePump();
	mRoot->renderOneFrame(10);

	// remove all nodes for next frame
	mSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();

	return mWindow->isClosed();
}

void OgreCanvas::drawBone(Vec3 start, Vec3 end, double radius) {
	drawVec3(start, end-start, radius);
};

void OgreCanvas::drawVec3(Vec3 origin, Vec3 vec, double radius) {
	if(vec.isApprox(Vec3(0,0,0))) {
		drawPoint(origin, radius);
	}
	else {
		// generate a mesh
		Procedural::CapsuleGenerator gen = Procedural::CapsuleGenerator();
		gen.setRadius(radius);
		gen.setHeight(vec.norm());

		// create an entity
		Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
		se->setMaterialName("Ogre/Earring");
		draw(se, origin + (vec * 0.5), yToDirQuat(vec));
	}
}

void OgreCanvas::drawPoint(Vec3 p, double radius) {
	// generate a mesh
	Procedural::SphereGenerator gen = Procedural::SphereGenerator();
	gen.setRadius(radius);

	// create an entity
	Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
	se->setMaterialName("Ogre/Earring");
	draw(se, p);
}
