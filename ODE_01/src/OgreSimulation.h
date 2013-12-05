#ifndef	_OGRESIMULATION_H
#define	_OGRESIMULATION_H	1

#include <OGRE/Ogre.h>

#include "Simulation.h"

using namespace std;

class OgreSimulation: public Simulation {

public:

	OgreSimulation(const char * bvhFile);

	void run();

private:

	vector< pair<Ogre::SceneNode*,Skeleton*> > nodeSkelPairs;

	Ogre::Root * mRoot;
	Ogre::Camera * mCamera;
	Ogre::SceneManager * mSceneMgr;
	Ogre::RenderWindow * mWindow;
	Ogre::SceneNode * sphereNode;

	void initOgre();
	void realizeSkeletons();
	void realizeSkeleton(Skeleton * s);
	void updateFromSim();
	void mainLoop();

};

#endif
