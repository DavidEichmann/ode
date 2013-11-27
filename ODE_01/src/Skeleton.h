
#ifndef	_SKELETON_H
#define	_SKELETON_H	1

#include <vector>

using namespace std;

class Skeleton {

public:

	vector<Skeleton *> children;
	Skeleton * parent;
	string name;	// name of the (root) joint

	// we assume that if there is no position chanel, then pos is just the offset
	bool hasPosChan;
	//bool hasRot = false;
	//double offset[3];

	double pos[3];	// of the start of the bone (bone extends to it's childrens' pos-es)
	double rot[3];	// relative to parent

	Skeleton();
	int calculateNumChan();
	int calculateContributingNumChan();

private:

};

#endif
