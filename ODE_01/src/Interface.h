
#ifndef	_INTERFACE_H
#define	_INTERFACE_H	1

extern "C" {

	void initOgre();

	void drawBone(double r,double g,double b,double a,   double startX,double startY,double startZ,double endX,double endY,double endZ, double radius);
	void drawVec3(double r,double g,double b,double a,   double originX,double originY,double originZ,double X,double Y,double Z, double radius);
	void drawPoint(double r,double g,double b,double a,   double X,double Y,double Z, double radius);
	bool doRender();

}


#endif
