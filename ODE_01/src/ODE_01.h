
#ifndef	_MAIN_H
#define	_MAIN_H	1

extern "C" {

	int c_main(int pargc, char** argv);

	void initOgre();

	void drawBone(double startX,double startY,double startZ,double endX,double endY,double endZ, double radius);
	void drawVec3(double originX,double originY,double originZ,double X,double Y,double Z, double radius);
	void drawPoint(double X,double Y,double Z, double radius);
	bool doRender();

}


#endif
