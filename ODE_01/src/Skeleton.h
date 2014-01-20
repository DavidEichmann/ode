
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

	// we assume that if there is no position chanel, then pos is just the offset
	bool hasPosChan;

	double pos[3];	// Local position of the start of the bone (bone extends to it's childrens' pos-es)
	double rot[3];	// rotation relative to parent (Z X Y: raw 3 axis rotation as found in BVH files)

	Skeleton();
	bool hasParent();
	Vec3 getPos();
	Vec3 getPosG();
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
	};

private:

	void updateRotQ();	// update local rotQ
	void updateGlobals();	// update local rotation quaternion
	Vec3 posG;
	Quat rotQG;
	Quat rotQ;

};

#endif
