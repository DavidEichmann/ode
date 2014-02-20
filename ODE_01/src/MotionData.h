
#ifndef	_MOTIONDATA_H
#define	_MOTIONDATA_H	1

#include <vector>

#include "Skeleton.h"

using namespace std;

class MotionData {

public:

	MotionData();
	MotionData(const char * filePath);
	~MotionData();
	virtual void parse(const char * filePath);
	vector<Skeleton*> getKeyframe(int index); // load the keyframe data into the skeleton object(s)
	double getFrameTime() { return frameTime; }
	int getNumFrames() { return numFrames; }

protected:

	vector< vector<Skeleton *> > frames;

private:

	vector<Skeleton *> baseSkeletons;
	int numChan;		// total number of channels
	int numFrames;		// total number of key frames
	double frameTime;
	char wordBuf[100];
	string nextWord(ifstream & in);

	void parse(ifstream & in);
	void parseKeyfames(ifstream & in);
	void parseHierarchy(ifstream & in);
	void parseRoot(ifstream & in);
	Skeleton * parseSkeleton(ifstream & in);
	vector<Skeleton*> cloneBaseSkeletons();

	int getNumChan();
};

#endif
