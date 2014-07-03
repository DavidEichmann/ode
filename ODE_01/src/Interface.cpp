#include "OgreCanvas.h"
#include <Eigen/Core>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/QR>
#include "Util.h"

#include "Interface.h"

// ffi functions


/*
 * Solve A x = b
 *
 * n	dimentionality of rhs vectors (also matrix is n x n)
 * nnz	number of non-empty (non-zero) matrix elements
 * mixs	array of matrix indexies  (in the form: [row1,col1,row2,col2 ... , row nz, col nz])
 * r	number of rhs vectors
 * vals	vals of non-empty matrix elements followed by rhs vectors
 *
 * returns r*n values representing solved vectors
 */
double* matrixSolveResults = new double[1];
double* sparseMatrixSolve(const int n, const int nnz, int* const mixs, const int r, double* const vals, bool leastSquare) {
	cout << "Sparse Matrix solve started..." << (leastSquare ? " (with least square)" : "")<< endl;
	double* valsC = vals;
	// prepare return buffer
	delete[] matrixSolveResults;
	matrixSolveResults = new double[n*r];
	// fill matrix A
	SparseMatrix<double,RowMajor> A(n,n);
	std::vector< Eigen::Triplet<double> > entries;
	int* mixsC = mixs;
	for(int i = 0; i < nnz; i++) {
		entries.push_back(Eigen::Triplet<double>(mixsC[0],mixsC[1],*valsC));
		valsC++;
		mixsC += 2;
	}
	A.setFromTriplets(entries.begin(),entries.end());
	A.makeCompressed();
	//cout << "Matrix determinant: " << A.toDense().determinant() << endl;
	// fill b
	MatrixXd b(n,r);
	MatrixXd x(n,r);
	for(int bc = 0; bc < r; bc++) {
		for(int br = 0; br < n; br++) {
			b(br,bc) = *valsC;
			valsC++;
		}
	}

//	cout << "vals\n";
//	for(int i = 0; i < nnz; i++) {
//		cout << vals[i] << " ";
//	}
//	cout << "\nmixs\n";
//	for(int i = 0; i < nnz*2; i++) {
//		cout << mixs[i] << " ";
//	}
	// solve A x = b
	if(leastSquare) {
		SparseQR< SparseMatrix<double>, AMDOrdering<int> > solver;
		x = A.toDense().fullPivHouseholderQr().solve(b);
//		solver.setPivotThreshold(0);
//		solver.compute(A);
//		if(solver.info()!=Success) {
//			// decomposition failed
//			cerr << "Matrix solver (with least square): failed in COMPUTE stage" << endl;
//			exit(1);
//		}
//		x = solver.solve(b);
//		if(solver.info()!=Success) {
//			// solving failed
//			cerr << "Matrix solver (with least square): failed in SOLVE stage" << endl;
//			exit(1);
//		}
	}
	else {
		SparseLU< SparseMatrix<double> > solver;
		solver.compute(A);
		if(solver.info()!=Success) {
			// decomposition failed
			cerr << "Matrix solver: failed in COMPUTE stage" << endl;
			exit(1);
		}
		x = solver.solve(b);
		if(solver.info()!=Success) {
			// solving failed
			cerr << "Matrix solver: failed in SOLVE stage" << endl;
			exit(1);
		}
	}
	// copy results into array
	for(int bc = 0; bc < r; bc++) {
		for(int br = 0; br < n; br++) {
			matrixSolveResults[(n*bc)+br] = x(br,bc);
		}
	}
	cout << "Sparse Matrix solve finished!" << endl;
	return matrixSolveResults;
}

double* matrixInverseResults = new double[1];
double* inverseMatrix(const int r, const int c, double* vals) {
	// prepare return buffer
	delete[] matrixInverseResults;
	matrixInverseResults = new double[r*c];
	MatrixXd m(r,c);
	MatrixXd mInv(r,c);
	for(int ri = 0; ri < r; ri++) {
		for(int ci = 0; ci < c; ci++) {
			m(ri,ci) = vals[(ri*c) + ci];
		}
	}
	FullPivLU<MatrixXd> lu(m);
	if(!lu.isInvertible()) {
		cout << "Matrix is NOT INVERTABLE!!! det: " << lu.determinant() << endl;
		cout << "Matrix: " << endl;
		for(int ri = 0; ri < r; ri++) {
			for(int ci = 0; ci < c; ci++) {
				cout << m(ri,ci) << "\t";
			}
			cout << endl;
		}
		exit(1);
	}
	mInv = lu.inverse();

	for(int ri = 0; ri < r; ri++) {
		for(int ci = 0; ci < c; ci++) {
			matrixInverseResults[(ri*c) + ci] = mInv(ri,ci);
		}
	}
	return matrixInverseResults;
}

OgreCanvas oc;

void initOgre() { oc.initOgre(); }

void drawPolygon(double r,double g,double b,double a, int n, double* const vals) {
	oc.drawPolygon(Ogre::ColourValue(r,g,b,a), n, vals);
}
void drawBox(double r,double g,double b,double a,   double sx,double sy,double sz, double cx,double cy,double cz,  double w,double x,double y, double z) {
	oc.drawBox(Ogre::ColourValue(r,g,b,a), Vec3(sx,sy,sz), Vec3(cx,cy,cz), Quat(w,x,y,z));
}
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



double timeStep;
double contactERP;
double contactCFM;
dWorldID wid;
dSpaceID sid;
dJointGroupID jointGroupid;
dJointGroupID contactGroupid;
vector<dContact> contacts;
vector<dBodyID> contactBodies;
double * buf = new double[200]; // pos + rot

double * getFloorContacts() {
	buf[0] = contacts.size();
	double* bufa = buf + 1;
	for(auto it = contacts.begin(); it != contacts.end(); it++) {
		bufa[0] = (*it).geom.pos[0];
		bufa[1] = (*it).geom.pos[2];
		bufa += 2;
	}
	return buf;
}

double * getBodyPosRot(dBodyID bid) {
	const dReal * pos = dBodyGetPosition(bid);
	const dReal * rot = dBodyGetQuaternion(bid);
	memcpy(buf,		pos,	3 * sizeof(double));
	memcpy(buf + 3,	rot,	4 * sizeof(double));
	return buf;
}

void setBodyPosRot(dBodyID bid, double x, double y, double z, double qw, double qx, double qy, double qz) {
	dBodySetPosition(bid,x,y,z);
	dQuaternion q{qw, qx, qy, qz};
	dBodySetQuaternion(bid,q);
}

double* getBodyGeom(dBodyID bid) {
	const int cls = dGeomGetClass(dBodyGetFirstGeom(bid));
	buf[0] = cls;
	buf++; // leave a space at the head of the buffer and get the geom info
	if(cls == dBoxClass) {
		getBodyGeomBox(bid);
	}
	else if(cls == dCapsuleClass) {
		getBodyGeomStartEnd(bid);
	}
	buf--; // reset buffer head
	return buf;
}

double* getBodyGeomStartEnd(dBodyID bid) {

	dGeomID g = dBodyGetFirstGeom(bid);
	const dReal * dq = dBodyGetQuaternion(bid);
	dReal doq[4]; dGeomGetOffsetQuaternion(g,doq);

	dReal radius;
	dReal length;
	dGeomCapsuleGetParams(g,&radius,&length);

	Quat qoq = eigQuat(dq) * eigQuat(doq);
	Vec3 pos = eigVec3(dBodyGetPosition(bid));

	Vec3 boneVec = qoq * Vec3(0,0,length/2);
	Vec3 start = pos + boneVec;
	Vec3 end   = pos - boneVec;

	buf[0] = start[0];
	buf[1] = start[1];
	buf[2] = start[2];
	buf[3] = end[0];
	buf[4] = end[1];
	buf[5] = end[2];
	return buf;
}

double* getBodyGeomBox(dBodyID bid) {

	dGeomID g = dBodyGetFirstGeom(bid);

	dReal lengths[4]; dGeomBoxGetLengths(g, lengths);

	Vec3 pos = eigVec3(dBodyGetPosition(bid));

	dReal doq[4]; dGeomGetOffsetQuaternion(g,doq);
	const dReal * dq = dBodyGetQuaternion(bid);
	Quat rot = eigQuat(dq) * eigQuat(doq);

	buf[0] = lengths[0];
	buf[1] = lengths[1];
	buf[2] = lengths[2];
	buf[3] = pos[0];
	buf[4] = pos[1];
	buf[5] = pos[2];
	buf[6] = rot.w();
	buf[7] = rot.x();
	buf[8] = rot.y();
	buf[9] = rot.z();
	return buf;
}

dWorldID initODE(double dt) {
	timeStep = dt;
	contactERP = 75.0 / ((75 * timeStep) + 2);
	contactCFM = 1.0 / ((75000 * timeStep) + 2000);
	dInitODE();
	//	Create a dynamics world.
	wid = dWorldCreate();
	// set CFM
	dWorldSetCFM(wid,0.00007);
	dWorldSetERP(wid,0.1);
	//	Create a joint group to hold the contact joints.
	jointGroupid = dJointGroupCreate(100);
	contactGroupid = dJointGroupCreate(1000);
	/// gravity
	dWorldSetGravity(wid, 0, -GRAVITY_ACC, 0);
	/// space
	sid = dHashSpaceCreate(0);
	/// floor
	dCreatePlane(sid, 0, 1, 0, 0);

	return wid;
}

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

) {

	// create a body for this bone
	dBodyID bid = dBodyCreate(wid);

	// NOTE that capsules are aligned along the Z axis
	dGeomID bGeom = dCreateCapsule(sid, radius, length);
	dGeomSetBody(bGeom, bid);

	// set the local rotation and offset
	dQuaternion q{qw,qx,qy,qz};
	dGeomSetOffsetQuaternion(bGeom, q);

	// add mass to the body
	dMass pMass;
	dBodyGetMass(bid, &pMass);
	dMassSetParameters (&pMass, mass,
							 0,0,0,
							 i11, i22, i33,
							 i12, i13, i23);
	dBodySetMass(bid, &pMass);

	// set the position and orientation of the body
	Vec3 gPos(x,y,z);
	dQuaternion gRot{rqw,rqx,rqy,rqz};
	dBodySetPosition(bid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);
	dBodySetQuaternion(bid, gRot);


	return bid;
}

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
) {

	// create a body for this bone
	dBodyID bid = dBodyCreate(wid);

	// NOTE that capsules are aligned along the Z axis
	dGeomID bGeom = dCreateBox(sid, sx,sy,sz);
	dGeomSetBody(bGeom, bid);

	// add mass to the body
	dMass pMass;
	dBodyGetMass(bid, &pMass);
	dMassSetParameters (&pMass, mass,
							 0,0,0,
							 i11, i22, i33,
							 i12, i13, i23);
	dBodySetMass(bid, &pMass);

	// set the position and orientation of the body
	Vec3 gPos(x,y,z);
	dQuaternion gRot{rqw,rqx,rqy,rqz};
	dBodySetPosition(bid,(dReal)gPos[0],(dReal)gPos[1],(dReal)gPos[2]);
	dBodySetQuaternion(bid, gRot);


	return bid;
}

void createBallJoint(dBodyID a, dBodyID b, double x, double y, double z) {
	dJointID jid = dJointCreateBall(wid,jointGroupid);
	dJointAttach(jid, a, b);
	dJointSetBallAnchor(jid, x, y, z);
}

// we use 3 axies: one to enforce the magnatude of angular velocity about the given axis,
// and the other 2 to constrain angular velocity to the angular velocity axis
dJointID createAMotor(dBodyID a, dBodyID b) {
	dJointID jid = dJointCreateAMotor(wid, jointGroupid);
	dJointAttach(jid, a, b);
	dJointSetAMotorMode(jid, dAMotorUser);
	dJointSetAMotorNumAxes(jid, 3);
	dJointSetAMotorParam(jid, dParamFMax,  dInfinity);
	dJointSetAMotorParam(jid, dParamFMax1, dInfinity);
	dJointSetAMotorParam(jid, dParamFMax2, dInfinity);
	dJointSetAMotorParam(jid, dParamFMax3, dInfinity);
	return jid;
}

void setAMotorVelocity(dJointID jid, double x, double y, double z) {
	if(x == 0 && y == 0 && z == 0) {
		dJointSetAMotorAxis(jid,0,0, 1,0,0);
		dJointSetAMotorAxis(jid,1,0, 0,1,0);
		dJointSetAMotorAxis(jid,2,0, 0,0,1);
		dJointSetAMotorParam(jid, dParamVel, 0);
		dJointSetAMotorParam(jid, dParamVel1, 0);
		dJointSetAMotorParam(jid, dParamVel2, 0);
		dJointSetAMotorParam(jid, dParamVel3, 0);
	}
	else {
		// axis 0 is the rotation axis
		Vec3 a1{x,y,z};
		dJointSetAMotorAxis(jid,0,0, (double) a1(0), (double) a1(1), (double) a1(2));
		dJointSetAMotorParam(jid, dParamVel1, a1.norm());
		// axis 1 and 2 are axies perpendicular to axis 0 such that all axis are orthogonal
		Vec3 a2;
		Vec3 a3;
		if(x == 0 && y == 0) {
			a2 = Vec3{1,0,0};
		}
		else {
			a2 = a1.cross(Vec3{x,y,z+1});
		}
		a3 = a1.cross(a2);
		dJointSetAMotorAxis(jid,1,0, (double) a2(0), (double) a2(1), (double) a2(2));
		dJointSetAMotorParam(jid, dParamVel2, 0);
		dJointSetAMotorAxis(jid,2,0, (double) a3(0), (double) a3(1), (double) a3(2));
		dJointSetAMotorParam(jid, dParamVel3, 0);
	}
}

void createFixedJoint(dBodyID a, dBodyID b) {
	dJointID jid = dJointCreateFixed(wid,jointGroupid);
	dJointAttach(jid, a, b);
	dJointSetFixed(jid);
}
void collisionCallback(void * data, dGeomID o1, dGeomID o2) {

	// ignore collisions with spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		return;
	}

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

//	// don't collide overlapping/connected bones
//	if(b1 != 0 && b2 != 0 && (dAreConnectedExcluding(b1,b2,dJointTypeContact) || overlap(b1,b2))) {
//		return;
//	}

	// only plane collisions
	if(dGeomGetClass(o1) != dPlaneClass && dGeomGetClass(o2) != dPlaneClass) {
		return;
	}
	// find the body that is not the floor
	dBodyID bb = (dGeomGetClass(o1) != dPlaneClass) ? b1 : b2;

	// collect collision info
	const int maxC = 10;
	dContactGeom contact[maxC];
	const int c = dCollide(o1, o2, maxC, contact, (int) sizeof(dContactGeom));

	// create collision joints
	for (int i = 0; i < c; i++) {
		if(contact[i].depth != 0) {
			dContact dc;

			dc.surface.mode = dContactRolling; // | dContactSoftERP | dContactSoftCFM;
			dc.surface.soft_erp = 0.1;
			dc.surface.soft_cfm = 0.00007;
			dc.surface.mu = dInfinity;
			dc.surface.rhoN = 1;
			dc.geom = contact[i];

			dJointID cj = dJointCreateContact(wid, contactGroupid, &dc);
			//dJointAttach(cj, b1, b2);

			// add point to vector
			contacts.push_back(dc);
			contactBodies.push_back(bb);
		}
	}
}

double * stepArgs = new double[3];
double atanBary(Vector2d a, Vector2d b, Vector2d c) {
	Vector3d ba;
	ba << (a - b), 0;
	Vector3d bc;
	bc << (c - b), 0;

	return (bc.dot(ba) / bc.cross(ba).norm());
}
void doCollisions() {
	// clear vectors to store contact points
	contacts.clear();
	contactBodies.clear();
	// do collision detection (filling contacts and contactBodies)
	dSpaceCollide(sid, stepArgs, &collisionCallback);

	double zmpX = stepArgs[0];
	double zmpZ = stepArgs[1];
	double fy   = stepArgs[2];

	// calculate the y forces to apply to each contact point
	// this method is described in http://davideichmann.com/wiki/Enforcing_CoP

	// using generalised barycentric coordinates
	Vector2d p{zmpX,zmpZ};
	int n = 4;

	double weightSum = 0;
	double w[n];

	contacts.push_back(contacts[0]);
	contacts.push_back(contacts[0]);
	contacts.push_back(contacts[0]);
	for(int j = 0; j < n; j++) {
		contacts[j].geom.pos[0] = (j <= 1) ? -50 : 50;
		contacts[j].geom.pos[1] = 0;
		contacts[j].geom.pos[2] = (1 <= j && j <= 2) ? -50 : 50;
	}

	for(int j = 0; j < n; j++) {

		int prev = (j + n - 1) % n;
		int next = (j + 1) % n;

		Vector2d qprev{contacts[prev].geom.pos[0],contacts[prev].geom.pos[2]};
		Vector2d qj{contacts[j].geom.pos[0],contacts[j].geom.pos[2]};
		Vector2d qnext{contacts[next].geom.pos[0],contacts[next].geom.pos[2]};

		w[j] = (atanBary(p,qj,qprev) + atanBary(p,qj,qnext)) / pow((p-qj).norm(),2);
		weightSum += w[j];
	}
	for(int j = 0; j < n; j++) {
		w[j] /= weightSum;
		double fj = w[j] * fy;
		cout << "y force on contact point " << j << ": " << fj << endl;
		dBodyAddForceAtPos(
				contactBodies[1],
				0, fj, 0,	// force to add
				contacts[j].geom.pos[0],
				contacts[j].geom.pos[1],
				contacts[j].geom.pos[2]);
	}

	/* psudo invers method is no longer used
	//   create matrix
	int cn = contacts.size();
	MatrixXd matrix(3,cn);
	//matrix.resize(NoChange, cn);
	for(int c = 0; c < cn; c++) {
		matrix(0,c) = contacts[c].geom.pos[0] / fy;
		matrix(1,c) = contacts[c].geom.pos[2] / fy;
		matrix(2,c) = 1;
	}

	//   create RHS vector
	Vector3d rhs;
	rhs(0) = zmpX;
	rhs(1) = zmpZ;
	rhs(2) = fy;

	//   solve for x in  M y = RHS  using psudoinverse: (M^T (M M^T)^-1) RHS = y
	MatrixXd matrixT = matrix.transpose();
	MatrixXd mmT = matrix * matrixT;
	//      find x in  mmt x = RHS
	VectorXd x = mmT.colPivHouseholderQr().solve(rhs);
	//      find the final answer for y
	MatrixXd y = matrixT * x;


	// apply the y forces
	cout << "applying forces" << endl;
	for(int i = 0; i < cn; i++) {
		cout << "y force on contact point " << i << ": " << y(i) << endl;
		dBodyAddForceAtPos(contactBodies[i],0, (double) y(i) ,0,contacts[i].geom.pos[0],contacts[i].geom.pos[1],contacts[i].geom.pos[2]);
	}
	 */
}
void step(dWorldID, double zmpX, double zmpZ, double fy) {
	stepArgs[0] = zmpX;
	stepArgs[1] = zmpZ;
	stepArgs[2] = fy;
	doCollisions();
	dWorldStep(wid,timeStep);
	// Remove all joints in the contact joint group.
	dJointGroupEmpty(contactGroupid);
}

