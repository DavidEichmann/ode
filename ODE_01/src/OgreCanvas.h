#ifndef	_OGRECANVAS_H
#define	_OGRECANVAS_H	1

#include "Util.h"
#include "Constants.h"
#include <OGRE/Ogre.h>

using namespace std;

class OgreCanvas {

public:

	OgreCanvas();

	void initOgre();

	Ogre::Root * mRoot;
	Ogre::Camera * mCamera;
	Ogre::SceneManager * mSceneMgr;
	Ogre::RenderWindow * mWindow;

	void draw(Ogre::Entity * e, Vec3 pos = Vec3::Zero(), Quat rot = Quat::Identity());
	virtual void drawPolygon(Ogre::ColourValue c, const int n, double* const vals);
	virtual void drawBox(Ogre::ColourValue c, Vec3 size, Vec3 center, Quat rotation);
	virtual void drawBone(Ogre::ColourValue c, Vec3 start, Vec3 end, double radius = BONE_RADIUS);
	virtual void drawVec3(Ogre::ColourValue c, Vec3 origin, Vec3 vec, double radius = 0.05);
	virtual void drawPoint(Ogre::ColourValue c, Vec3 p, double radius = 0.07);
	virtual void drawBone(Vec3 start, Vec3 end, double radius = BONE_RADIUS);
	virtual void drawVec3(Vec3 origin, Vec3 vec, double radius = 0.05);
	virtual void drawPoint(Vec3 p, double radius = 0.07);
	Ogre::MaterialPtr createMaterial(Ogre::ColourValue c);
	bool doRender();

};

#endif
