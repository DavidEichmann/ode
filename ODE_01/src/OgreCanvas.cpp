
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
	mCamera->setPosition(Ogre::Vector3(0, 1, 5));
	mCamera->lookAt(Ogre::Vector3(0, 1, 0));
	mCamera->setNearClipDistance(0.05);
	mCamera->setFarClipDistance(100);

	/// create window/viewport
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0.7, 0.7, 1));
	// Alter the camera aspect ratio to match the viewport
	mCamera->setAspectRatio(
			Ogre::Real(vp->getActualWidth())
					/ Ogre::Real(vp->getActualHeight()));

	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	/// light
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	Ogre::Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(200, 200, 300);


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

int manualObjectCount = 0;
Ogre::ManualObject manual("ManualOutline");
Ogre::ManualObject manualR("ManualOutlineReverse");
void OgreCanvas::drawPolygon(Ogre::ColourValue c, const int n, double* const vals) {
	if(n > 0) {
		manual.clear();
		manualR.clear();

		manual.begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN);
		manualR.begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN);

		for (int j = 0; j < n; j++)
		{
			int baseIx = j*3;
			int baseIxR = ((3*n)-3) - (j*3);
			double posAX = vals[baseIx];
			double posAY = vals[baseIx+1];
			double posAZ = vals[baseIx+2];
			double posAXR = vals[baseIxR];
			double posAYR = vals[baseIxR+1];
			double posAZR = vals[baseIxR+2];
			manual.position(posAX, posAY, posAZ);
			manualR.position(posAXR, posAYR, posAZR);
		}

		for (int j = 0; j < n - 1; j++)
		{
			  manual.index(j);
			  manualR.index(j);
		}

		manual.end();
		manualR.end();

		Ogre::MeshPtr ptr = manual.convertToMesh("ManualOutline"+(manualObjectCount++));
		Ogre::MeshPtr ptrR = manualR.convertToMesh("ManualOutlineR"+(manualObjectCount++));

		Ogre::Entity* myEntity = mSceneMgr->createEntity(ptr);
		Ogre::Entity* myEntityR = mSceneMgr->createEntity(ptrR);

		Ogre::MaterialPtr mat = createMaterial(c);
		myEntity->setMaterial(mat);
		myEntityR->setMaterial(mat);

		draw(myEntity, Vec3(0,0,0), Quaterniond::Identity());
		draw(myEntityR, Vec3(0,0,0), Quaterniond::Identity());
	}
}

void OgreCanvas::drawBone(Ogre::ColourValue c, Vec3 start, Vec3 end, double radius) {
	drawVec3(c, start, end-start, radius);
};

void OgreCanvas::drawVec3(Ogre::ColourValue c, Vec3 origin, Vec3 vec, double radius) {
//	if(isnan((double) vec(0)) || isnan((double) vec(1)) || isnan((double) vec(2)) || isnan((double) origin(0)) || isnan((double) origin(1)) || isnan((double) origin(2))) {
//		cout << "Error: Attempting to draw a vector with NAN value (origin, vec):\n\t";
//		print(origin);
//		cout << "\t";
//		print(vec);
//		return;
//	}
	if(radius <= 0.00001) {
		return;
	}
	if(vec.isApprox(Vec3(0,0,0))) {
		drawPoint(c, origin, radius);
	}
	else {
		// generate a mesh
		Procedural::CapsuleGenerator gen = Procedural::CapsuleGenerator();
		gen.setRadius(radius);
		gen.setHeight(vec.norm());

		// create an entity
		Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
		//se->setMaterialName("Ogre/Earring");
		se->setMaterial(createMaterial(c));
		draw(se, origin + (vec * 0.5), yToDirQuat(vec));
	}
}

void OgreCanvas::drawPoint(Ogre::ColourValue c, Vec3 p, double radius) {
	// generate a mesh
	Procedural::SphereGenerator gen = Procedural::SphereGenerator();
	gen.setRadius(radius);

	// create an entity
	Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
	se->setMaterial(createMaterial(c));
	draw(se, p);
}

void OgreCanvas::drawBox(Ogre::ColourValue c, Vec3 size, Vec3 center, Quat rotation) {
	// generate a mesh
	Procedural::BoxGenerator gen = Procedural::BoxGenerator();
	gen.setSize(ogreConv(size));

	// create an entity
	Ogre::Entity * se = mSceneMgr->createEntity(gen.realizeMesh());
	se->setMaterial(createMaterial(c));
	draw(se, center, rotation);
}

void OgreCanvas::drawBone(Vec3 start, Vec3 end, double radius) { drawBone(Ogre::ColourValue::White, start, end, radius); }
void OgreCanvas::drawVec3(Vec3 origin, Vec3 vec, double radius) { drawVec3(Ogre::ColourValue::White, origin, vec, radius); }
void OgreCanvas::drawPoint(Vec3 p, double radius) { drawPoint(Ogre::ColourValue::White, p, radius); }

// Ogre::ColourValue(red, green, blue, alpha)
Ogre::MaterialPtr OgreCanvas::createMaterial(Ogre::ColourValue c) {

	Ogre::MaterialPtr mMat = Ogre::MaterialManager::getSingleton().create("", "General", false);
	Ogre::Technique* mTech = mMat->createTechnique();
	Ogre::Pass* mPass = mTech->createPass();
	Ogre::TextureUnitState* mTexUnitState = mPass->createTextureUnitState();
	mPass = mMat->getTechnique(0)->getPass(0);

	mPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);	// allow transparency
	mPass->setDiffuse(c);
	mPass->setAmbient(c * 0.4);
	mPass->setSpecular(1,1,1,c.a);

	return mMat;
}
