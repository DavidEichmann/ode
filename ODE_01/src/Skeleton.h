
#ifndef	_SKELETON_H
#define	_SKELETON_H	1

#include <OgreSceneNode.h>
#include <vector>

using namespace std;

class Skeleton {

public:

	static const string END_SITE;

	vector<Skeleton *> children;
	Skeleton * parent;
	string name;	// name of the (root) joint
	Ogre::SceneNode * ogreNode;

	// we assume that if there is no position chanel, then pos is just the offset
	bool hasPosChan;
	//bool hasRot = false;
	//double offset[3];

	double pos[3];	// of the start of the bone (bone extends to it's childrens' pos-es)
	double rot[3];	// rotation relative to parent

	Skeleton();
	bool hasParent();
	bool hasOgreNode();
	const double * getPosXYZ();
	const double * getRotXYZ();
	int calculateNumChan();
	int calculateContributingNumChan();

private:
	double posXYZ[3];
	double rotXYZ[3];

};

#endif
