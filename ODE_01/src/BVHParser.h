
#ifndef	_BVHPARSER_H
#define	_BVHPARSER_H	1

#include <vector>

#include "Skeleton.h"

using namespace std;

class BVHParser {

public:

	vector<Skeleton *> skeletons;
	int numChan;		// total number of channels
	int numFrames;		// total number of key frames
	double frameTime;

	BVHParser(const char * filePath);
	void loadKeyframe(int index); // load the keyframe data into the skeleton object(s)

private:

	double * * keyframes;	// [frame] [channelvalue]
	double * * channels;	// convenient pointers to channel values in the skeleton
	char wordBuf[100];
	string nextWord(ifstream & in);

	void parse(ifstream & in);
	void parseKeyfames(ifstream & in);
	void parseHierarchy(ifstream & in);
	void parseRoot(ifstream & in);
	Skeleton * parseSkeleton(ifstream & in);
	void calculateNumChan();
	void fillChannelsArray();
	void fillChannelsArray(Skeleton * s, double * * & nextCPos);
};

#endif
