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

Vec3 Skeleton::getOffset() {
	return Vec3(offset[0],offset[1],offset[2]);
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
	posG = pPos + (pQ * getOffset());
	rotQG = pQ * getRot();
}

void Skeleton::calculateMinMaxY(float & mi, float & ma) {
	Vec3 pos = getPosG();
	mi = min(mi, (float) pos[1]);
	ma = max(ma, (float) pos[1]);
	forall(children, [&](Skeleton * sk) {
		sk->calculateMinMaxY(mi,ma);
	});
}

float Skeleton::calculateScaleAndTranslate() {
	float mi,ma;
	Vec3 pos = getPosG();
	mi = ma = pos[1];
	calculateMinMaxY(mi,ma);
	float targetHeight = 1.75;
	float rawHeight = ma - mi;
	float scaleFactor = targetHeight/rawHeight;
	float translateY = 0.2 - mi;
	for(Skeleton* s : getAllSkeletons()) {
		s->scaleFactor = scaleFactor;
		s->translateY = translateY;
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




