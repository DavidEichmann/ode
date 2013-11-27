#include <string.h>
#include <iostream>
#include <fstream>

#include "Skeleton.h"

using namespace std;

Skeleton::Skeleton() {
	hasPosChan = false;
	parent = NULL;
}

int Skeleton::calculateNumChan() {
	int count = 0;

	// this links channels
	count += hasPosChan?6:3;

	// recurse to children
	long n = children.size();
	if( ! children.empty()) {
		for(vector<Skeleton *>::iterator it = children.begin(); it != children.end(); ++it) {
			count += (*it)->calculateNumChan();
		}
	}

	return count;
}

int Skeleton::calculateContributingNumChan() {
	return hasPosChan?6:3;
}
