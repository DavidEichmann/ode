
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
	float scaleFactor = 1;
	float translateY = 0;

	// we assume that if there is no position chanel, then pos is just the offset
	bool hasPosChan;

	double offset[3];	// offset from parent joint (in the local coordinate system of the parent joint)
	double rot[3];	// rotation relative to parent (Z X Y: raw 3 axis rotation as found in BVH files)

	Skeleton();
	bool hasParent();
	Vec3 getOffset();
	Vec3 getPosG();
	Vec3 getPosStart();
	Vec3 getPosStartG();
	Vec3 getPosEnd();
	Vec3 getPosEndG();
	Vec3 getPosCom();
	Vec3 getPosComG();
	Quat getRot();
	Quat getRotG();
	int calculateNumChan();
	int calculateContributingNumChan();
	// update values after rot and or pos have been changed
	// this is recursive
	void update() {
		updateRotQ();
		updateGlobals();
		for(vector<Skeleton*>::iterator it = children.begin(); it!= children.end(); it++) {
			(*it)->update();
		}
		if(!hasParent()) {
			float targetHeight = 1.75;
			float rawHeight = calculateScaleAndTranslate();
			scaleAndTranslate();
		}
	};
	vector<Skeleton*> getAllSkeletons();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

	void updateRotQ();	// update local rotQ
	void updateGlobals();	// update local rotation quaternion
	void scaleAndTranslate() {
		for(int i = 0; i < 3; i++) { offset[i] *= scaleFactor; }
		posG[1] += translateY;
		posG = posG * scaleFactor;
		forall(children, [&](Skeleton * c){ c->scaleAndTranslate(); });
	};
	Vec3 posG;
	Quat rotQG;
	Quat rotQ;

	void calculateMinMaxY(float & mi, float & ma);
	float calculateScaleAndTranslate();


};

#endif
