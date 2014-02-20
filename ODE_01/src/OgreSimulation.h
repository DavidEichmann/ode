#ifndef	_OGRESIMULATION_H
#define	_OGRESIMULATION_H	1

#include <OGRE/Ogre.h>
#include <map>

#include "Simulation.h"
#include "Human.h"

using namespace std;

class OgreSimulation: public Simulation {

public:

	OgreSimulation();
	OgreSimulation(const char * bvhFile);

	void run();

private:

	Ogre::Root * mRoot;
	Ogre::Camera * mCamera;
	Ogre::SceneManager * mSceneMgr;
	Ogre::RenderWindow * mWindow;

	Human * human = NULL;
	Ogre::SceneNode * * humanNodes;

	void initOgre();
	void realizeHuman();
	void realizeSkeletons();
	void mainLoop();

	void draw();
	void draw(Ogre::Entity * e, Vec3 pos = Vec3::Zero(), Quat rot = Quat::Identity());
	virtual void drawBone(Vec3 start, Vec3 end, double radius = BONE_RADIUS);
	virtual void drawVec3(Vec3 origin, Vec3 vec, double radius = 0.05);
	virtual void drawPoint(Vec3 p, double radius = 0.07);
	void doRender();

};

#endif
