
#ifndef	_SKELETON_H
#define	_SKELETON_H	1

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Util.h"

using namespace std;
using namespace Eigen;

class Skeleton {

public:

	static const string END_SITE;

	vector<Skeleton *> children;
	Skeleton * parent;
	string name;	// name of the (root) joint
	float scaleFactor = -1;
	Vec3 translate;

	// we assume that if there is no position chanel, then pos is just the offset
	bool hasPosChan;

	double offset[3];	// offset from parent joint (in the local coordinate system of the parent joint)
	double rot[3];	// rotation relative to parent (Z X Y: raw 3 axis rotation as found in BVH files)


	Skeleton();
	~Skeleton();
	Skeleton * clone();
	bool hasParent();
	Vec3 getOffset();
	double getLength();
	Vec3 getPosG();
	Vec3 getPosStart();
	Vec3 getPosStartG();
	Vec3 getPosEnd();
	Vec3 getPosEndG();
	Vec3 getPosCom();
	Vec3 getPosComG();
	Quat getRot();
	Quat getRotG();
	vector<Skeleton*> getAllSkeletons();
	double getMass();
	double getTotalMass();
	Vec3 getLinearVel() { return linearVel; }
	void setLinearVel(Vec3 lv) { linearVel = lv; }
	Vec3 getLinearAcc() { return linearAcc; }
	void setLinearAcc(Vec3 la) { linearAcc = la; }
	Vec3 getLinearMomentum();
	Vec3 getTotalLinearMomentum();
	Vec3 getTotalLinearMomentum_deriv();
	Vec3 getAngularVel() { return angularVel; }
	void setAngularVel(Vec3 av) { angularVel = av; }
	Vec3 getAngularAcc() { return angularAcc; }
	void setAngularAcc(Vec3 av) { angularAcc = av; }
	Vec3 getAngularMomentum();
	Vec3 getAngularMomentum_deriv();
	Vec3 getTotalAngularMomentum();
	Vec3 getTotalAngularMomentum_deriv();
	Matrix3d getInertiaTensor();
	string getLongName() {
		if(hasParent())
			return parent->name + " -> " + name;
		else
			return "(0,0,0) -> " + name;
	}
	int calculateNumChan();
	int calculateContributingNumChan();
	// update values after rot and or pos have been changed
	// this is recursive
	void update() {
		updateRotQ();
		updateGlobals();
	};
	void calculateScaleAndTranslate();
	Skeleton * getRoot() {
		if(hasParent())
			return parent->getRoot();
		else
			return this;
	};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

	Vec3 posG;
	Quat rotQG;
	Quat rotQ;
	double mass = -1;
	Vec3 linearVel{0,0,0};
	Vec3 linearAcc{0,0,0};
	Vec3 angularVel{0,0,0};
	Vec3 angularAcc{0,0,0};

	Vec3 getRawOffset();
	void updateRotQ();	// update local rotQ
	void updateGlobals();	// update local rotation quaternion
	void calculateMinMax(Vec3 & mi, Vec3 & ma);
};

#endif
