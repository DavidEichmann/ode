
#include <string.h>
#include <iostream>
#include <boost/chrono/system_clocks.hpp>
#include <time.h>
#include <ode/ode.h>
#include <OGRE/Ogre.h>
#include <OGRE/OgreFrameListener.h>
#include <OIS/OIS.h>
#include <Eigen/Geometry>
#include <tuple>

#include "Simulation.h"
#include "Constants.h"
#include "Util.h"
#include "MotionData.h"
#include "Procedural.h"
#include "OgreCanvas.h"


class SimpleFrameListener : public Ogre::FrameListener
{
public:
    SimpleFrameListener(OIS::Keyboard* keyboard, OIS::Mouse* mouse, Ogre::Camera* camera)
    {
        mKeyboard = keyboard;
        mMouse = mouse;
        mCam = camera;
    }
    // This gets called before the next frame is beeing rendered.
    bool frameStarted(const Ogre::FrameEvent& evt)
    {
        //update the input devices
        mKeyboard->capture();
        mMouse->capture();

        //exit if key KC_ESCAPE pressed
        if(mKeyboard->isKeyDown(OIS::KC_ESCAPE))
            return false;

        // move cam
        Ogre::Vector3 camdir = mCam->getDerivedDirection();
        Ogre::Vector3 campos = mCam->getPosition();
        double speed = 0.1;
        double mspeed = 0.001;
        if(mKeyboard->isKeyDown(OIS::KC_A))
        	mCam->setPosition(campos + (camdir.crossProduct(Ogre::Vector3::UNIT_Y).normalisedCopy() * -speed));
        if(mKeyboard->isKeyDown(OIS::KC_D))
        	mCam->setPosition(campos + (camdir.crossProduct(Ogre::Vector3::UNIT_Y).normalisedCopy() * speed));
        if(mKeyboard->isKeyDown(OIS::KC_W))
        	mCam->setPosition(campos + (camdir.normalisedCopy() * speed));
        if(mKeyboard->isKeyDown(OIS::KC_S))
        	mCam->setPosition(campos + (camdir.normalisedCopy() * -speed));

        Vec3 nd = ((Quaterniond) AngleAxisd(
        		mMouse->getMouseState().X.rel * -mspeed, Vec3::UnitY()))
        				* eigConv(camdir);
		mCam->setDirection(
			ogreConv(
				nd
			)
		);

        return true;
    }
    // This gets called at the end of a frame.
    bool frameEnded(const Ogre::FrameEvent& evt)
    {
        return true;
    }
private:
    OIS::Keyboard* mKeyboard;
    OIS::Mouse* mMouse;
    Ogre::Camera* mCam;
};


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


	// setup OIS input
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;

	//tell OIS about the Ogre window
	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

	//setup the manager, keyboard and mouse to handle input
	OIS::InputManager* inputManager = OIS::InputManager::createInputSystem(pl);
	OIS::Keyboard* keyboard = static_cast<OIS::Keyboard*>(inputManager->createInputObject(OIS::OISKeyboard, true));
	OIS::Mouse*    mouse = static_cast<OIS::Mouse*>(inputManager->createInputObject(OIS::OISMouse, true));

	//tell OIS about the window's dimensions
	unsigned int width, height, depth;
	int top, left;
	mWindow->getMetrics(width, height, depth, left, top);
	const OIS::MouseState &ms = mouse->getMouseState();
	ms.width = width;
	ms.height = height;

	// everything is set up, now we listen for input and frames (replaces while loops)
//	//key events
//	SimpleKeyListener* keyListener = new SimpleKeyListener();
//	keyboard->setEventCallback(keyListener);
//	//mouse events
//	SimpleMouseListener* mouseListener = new SimpleMouseListener();
//	mouse->setEventCallback(mouseListener);
	//render events
	SimpleFrameListener* frameListener = new SimpleFrameListener(keyboard, mouse, mCamera);
	mRoot->addFrameListener(frameListener);
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

	return ! mWindow->isActive();
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
