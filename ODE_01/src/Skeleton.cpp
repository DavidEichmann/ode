#include <string.h>
#include <iostream>
#include <fstream>

#include "Constants.h"
#include "Util.h"
#include "Skeleton.h"
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

const string Skeleton::END_SITE = "End_Site";

Skeleton::Skeleton() {
	hasPosChan = false;
	parent = NULL;
}

bool Skeleton::hasParent() {
	return parent != NULL;
}

Vec3 Skeleton::getPos() {
	return Vec3(pos[0],pos[1],pos[2]);
}

Quat Skeleton::getRot() {
	return rotQ;
}

Vec3 Skeleton::getPosG() {
	return posG;
}

Quat Skeleton::getRotG() {
	return rotQG;
}

int Skeleton::calculateNumChan() {
	int count = 0;

	// this links channels
	count += calculateContributingNumChan();

	// recurse to children
	if( ! children.empty()) {
		for(vector<Skeleton *>::iterator it = children.begin(); it != children.end(); ++it) {
			count += (*it)->calculateNumChan();
		}
	}

	return count;
}

int Skeleton::calculateContributingNumChan() {
	if(name == Skeleton::END_SITE) { return 0; }
	return hasPosChan?6:3;
}

void Skeleton::updateRotQ() {
	// done in BVH style (rotate Y the X then Z)
	rotQ = 	AngleAxisd(D2R(rot[0]), Vec3::UnitZ()) *
			AngleAxisd(D2R(rot[1]), Vec3::UnitX()) *
			AngleAxisd(D2R(rot[2]), Vec3::UnitY());
}

void Skeleton::updateGlobals() {
	// get parent Global pos and rot
	Quat pQ;
	Vec3 pPos;
	if(hasParent()) {
		pQ = parent->getRotG();
		pPos = parent->getPosG();
	}
	else {
		pQ = AngleAxisd(0,Vec3(0,0,1));
		pPos = Vec3(0,0,0);
	}

	// apply parents global rot/pos to mine
	posG = pPos + (pQ * getPos());
	rotQG = pQ * getRot();
}






















