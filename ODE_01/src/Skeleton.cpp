#include <string.h>
#include <iostream>
#include <fstream>

#include "Skeleton.h"

using namespace std;

const string Skeleton::END_SITE = "End_Site";

Skeleton::Skeleton() {
	hasPosChan = false;
	parent = NULL;
	ogreNode = NULL;
}

bool Skeleton::hasParent() {
	return parent != NULL;
}

bool Skeleton::hasOgreNode() {
	return ogreNode != NULL;
}
const double * Skeleton::getPosXYZ() {
	posXYZ[0] = pos[0];
	posXYZ[1] = pos[1];
	posXYZ[2] = pos[2];
	return posXYZ;
}

const double * Skeleton::getRotXYZ() {
	// rot is in order: ZXY
	rotXYZ[0] = rot[1];
	rotXYZ[1] = rot[2];
	rotXYZ[2] = rot[0];
	return rotXYZ;
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
	//cout << Skeleton::END_SITE << endl;
	if(name == Skeleton::END_SITE) { return 0; }
	return hasPosChan?6:3;
}
