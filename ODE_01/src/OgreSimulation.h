#ifndef	_OGRESIMULATION_H
#define	_OGRESIMULATION_H	1

#include <OGRE/Ogre.h>
#include <map>

#include "Simulation.h"
#include "OgreCanvas.h"
#include "Human.h"

using namespace std;

class OgreSimulation : public Simulation, public OgreCanvas {

public:

	OgreSimulation();
	OgreSimulation(const char * bvhFile);

	void run();

private:

	Human * human = NULL;
	Ogre::SceneNode * * humanNodes;

	void initOgre();
	void realizeHuman();
	void realizeSkeletons();
	void mainLoop();

	void draw();
	void draw(Ogre::Entity * e, Vec3 pos = Vec3::Zero(), Quat rot = Quat::Identity()) {
		OgreCanvas::draw(e,pos,rot);
	};
	virtual void drawBone(Vec3 start, Vec3 end, double radius = BONE_RADIUS) {
		OgreCanvas::drawBone(start,end,radius);
	};
	virtual void drawVec3(Vec3 origin, Vec3 vec, double radius = 0.05) {
		OgreCanvas::drawVec3(origin, vec, radius);
	};
	virtual void drawPoint(Vec3 p, double radius = 0.07) {
		OgreCanvas::drawPoint(p, radius);
	};
	bool doRender() {
		return OgreCanvas::doRender();
	}

};

#endif
