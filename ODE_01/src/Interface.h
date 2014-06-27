
#ifndef	_INTERFACE_H
#define	_INTERFACE_H	1

extern "C" {

	//
	// Matrix solver
	//
	double* sparseMatrixSolve(int n, int nz, int* mixs, int r, double* vals);
	double* inverseMatrix(int r, int c, double* vals);


	//
	// OGRE
	//

	void initOgre();

	void drawPolygon(double r,double g,double b,double a,   int n, double* const vals);
	void drawBox    (double r,double g,double b,double a,   double sx,double sy,double sz, double cx,double cy,double cz,  double w,double x,double y, double z);
	void drawBone   (double r,double g,double b,double a,   double startX,double startY,double startZ,double endX,double endY,double endZ, double radius);
	void drawVec3   (double r,double g,double b,double a,   double originX,double originY,double originZ,double X,double Y,double Z, double radius);
	void drawPoint  (double r,double g,double b,double a,   double X,double Y,double Z, double radius);
	bool doRender();


	//
	// ODE
	//

	dWorldID initODE(double dt);

	double* getBodyGeomStartEnd(dBodyID bid);
	double* getBodyGeomBox(dBodyID bid);
	double* getBodyGeom(dBodyID bid);
	double* getBodyPosRot(dBodyID bid);
	void setBodyPosRot(dBodyID bid, double x, double y, double z, double qw, double qx, double qy, double qz);
	double* getBodyStartEnd(dBodyID bid);
	dBodyID appendCapsuleBody(

		// position CoM
		double x, double y, double z,

		// local rotation for the bone from Z aligned
		double qw, double qx, double qy, double qz,
		// global quaternion rotation
		double rqw, double rqx, double rqy, double rqz,

		// dimensions
		double radius,
		double length,

		// mass
		double mass,

		// Inertia matrix (about CoM)
		double i11, double i12, double i13,
					double i22, double i23,
								double i33
	);
	dBodyID appendFootBody(

		// position CoM
		double x, double y, double z,

		// global quaternion rotation
		double rqw, double rqx, double rqy, double rqz,

		// box size
		double sx, double sy, double sz,

		// mass
		double mass,

		// Inertia matrix (about CoM)
		double i11, double i12, double i13,
					double i22, double i23,
								double i33
	);
	void step(dWorldID wid, double zmpX, double zmpZ, double fy);
	void createBallJoint(dBodyID a, dBodyID b, double x, double y, double z);
	dJointID createAMotor(dBodyID a, dBodyID b);
	void setAMotorVelocity(dJointID jid, double x, double y, double z);
	void createFixedJoint(dBodyID a, dBodyID b);
}



#endif
