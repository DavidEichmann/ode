#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

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

Vec3 Skeleton::getRawOffset() {
	return Vec3(offset[0], offset[1], offset[2]);
}

Vec3 Skeleton::getOffset() {
	return getRawOffset() * scaleFactor;
}

Vec3 Skeleton::getPosStart() {
	return getOffset() * -1;
}

Vec3 Skeleton::getPosStartG() {
	if(hasParent()) { return parent->getPosG(); }
	return Vec3(0,0,0);
}

Vec3 Skeleton::getPosEndG() {
	return getPosG();
}

Vec3 Skeleton::getPosEnd() {
	return Vec3(0,0,0);
}

Vec3 Skeleton::getPosCom() {
	if(hasParent()) { return getPosStart() * 0.5; }
	return Vec3(0,0,0);
}

Vec3 Skeleton::getPosComG() {
	return (getPosStartG() + getPosEndG()) * 0.5;
}


Quat Skeleton::getRot() {
	return rotQ;
}

Vec3 Skeleton::getPosG() {
	return (posG + translate) * scaleFactor;
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
		for(Skeleton * c : children) {
			count += c->calculateNumChan();
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
//	rotQ = 	AngleAxisd(D2R(rot[0]), Vec3::UnitZ()) *
//			AngleAxisd(D2R(rot[1]), Vec3::UnitX()) *
//			AngleAxisd(D2R(rot[2]), Vec3::UnitY());
	rotQ = 	AngleAxisd(D2R(rot[0]), Vec3::UnitY()) *
			AngleAxisd(D2R(rot[1]), Vec3::UnitX()) *
			AngleAxisd(D2R(rot[2]), Vec3::UnitZ());
}
void Skeleton::updateGlobals() {
	// get parent Global pos and rot
	Quat pQ;
	Vec3 pPos;
	if(hasParent()) {
		pQ = parent->rotQG;
		pPos = parent->posG;
	}
	else {
		pQ = AngleAxisd(0,Vec3(0,0,1));
		pPos = Vec3(0,0,0);
	}

	// apply parents global rot/pos to mine
	posG = pPos + (pQ * getRawOffset());
	rotQG = pQ * getRot();

	// recurse to children
	for(Skeleton* c : children) {
		c->update();
	}
}

void Skeleton::calculateMinMax(Vec3 & mi, Vec3 & ma) {
	for(int i = 0; i < 3; i++) {
		mi[i] = min((float) mi[i], (float) posG[i]);
		ma[i] = max((float) ma[i], (float) posG[i]);
	}
	forall(children, [&](Skeleton * sk) {
		sk->calculateMinMax(mi,ma);
	});
}

void Skeleton::calculateScaleAndTranslate() {
	Vec3 mi,ma;
	Vec3 pos = posG;
	mi = ma = pos;
	calculateMinMax(mi,ma);
//	float targetHeight = 1.75;
//	float rawHeight = ma[1] - mi[1];
//	float scaleFactor = targetHeight/rawHeight;
	float scaleFactor = BVH_SCALE;
	Vec3 t = (ma + mi) / -2;
	t += Vec3(0, (0.2/scaleFactor) + ((ma-mi)[1]/2), 0);
	for(Skeleton* s : getAllSkeletons()) {
		s->scaleFactor = scaleFactor;
		s->translate = t;
	}
}

vector<Skeleton*> Skeleton::getAllSkeletons() {
	vector<Skeleton*> v;
	v.push_back(this);
	for(Skeleton* c : children) {
		for(Skeleton* cs : c->getAllSkeletons()) {
			v.push_back(cs);
		}
	}
	return v;
}




