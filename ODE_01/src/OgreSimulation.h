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

	vector< pair<Ogre::SceneNode*,Vector2i> > nodeSkelIndexPairs;
	map<Ogre::SceneNode*,Ogre::SceneNode*> nodeAniNodeMap;

	Ogre::Root * mRoot;
	Ogre::Camera * mCamera;
	Ogre::SceneManager * mSceneMgr;
	Ogre::RenderWindow * mWindow;
	Ogre::SceneNode * sphereNode;
	Ogre::SceneNode* ballNode = nullptr;

	Human * human = NULL;
	Ogre::SceneNode * * humanNodes;

	void initOgre();
	void realizeBall();
	void realizeHuman();
	void realizeSkeletons();
	void updateFromSim();
	void mainLoop();

};

#endif
