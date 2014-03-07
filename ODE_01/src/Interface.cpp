#include "OgreCanvas.h"
#include <Eigen/Core>
#include "Util.h"

#include "Interface.h"

// ffi functions

OgreCanvas oc;

void initOgre() { oc.initOgre(); }
void drawBone(double r,double g,double b,double a,   double startX,double startY,double startZ,double endX,double endY,double endZ, double radius) {
	oc.drawBone(Ogre::ColourValue(r,g,b,a), Vec3(startX, startY, startZ), Vec3(endX, endY, endZ), radius);
}
void drawVec3(double r,double g,double b,double a,   double originX,double originY,double originZ,double X,double Y,double Z, double radius) {
	oc.drawVec3(Ogre::ColourValue(r,g,b,a), Vec3(originX, originY, originZ), Vec3(X, Y, Z), radius);
}
void drawPoint(double r,double g,double b,double a,   double X,double Y,double Z, double radius) {
	oc.drawPoint(Ogre::ColourValue(r,g,b,a), Vec3(X,Y,Z), radius);
}
bool doRender() {
	return oc.doRender();
}
