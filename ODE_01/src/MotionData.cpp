#include <string.h>
#include <iostream>
#include <fstream>

#include "MotionData.h"
#include "Skeleton.h"

using namespace std;

MotionData::MotionData(const char * filePath) {
	parse(filePath);
}

MotionData::MotionData() {
}

MotionData::~MotionData() {
}

string MotionData::nextWord(ifstream & in) {
	// read next word
	in >> wordBuf;
	return string(wordBuf);
}

int MotionData::getNumChan() {
	// cached result
	numChan = 0;
	// calculate
	if( ! baseSkeletons.empty()) {
		for(Skeleton * s : baseSkeletons) {
			numChan += s->calculateNumChan();
		}
	}
	return numChan;
}

vector<Skeleton*> MotionData::getKeyframe(int index) {
	return frames[index];
}

void MotionData::parse(const char * filePath) {
	ifstream in;
	in.open(filePath, ios::in);
	parse(in);
}

void MotionData::parse(ifstream & in) {
	// heirarchy
	nextWord(in); // HEIRARCHY
	parseHierarchy(in);
//	updateSkeletons();

	// parse the keyframes
	parseKeyfames(in);
}

void MotionData::parseKeyfames(ifstream & in) {

	// count channels
	int numChan = getNumChan();

	// MOTION
	nextWord(in);

	// Frames:	765
	nextWord(in);
	in >> numFrames;

	// Frame Time:	0.0333333
	nextWord(in); nextWord(in);
	in >> frameTime;

	// keep reading values adding them to keyframes
	for(int f = 0; f < numFrames; f++) {
		vector<Skeleton*> frame = cloneBaseSkeletons();
		for(Skeleton * ss : frame) {
			for(Skeleton * s : ss->getAllSkeletons()) {
				// either XYZZXY (pos + rot) or just ZXY (rot)
				int nC = s->calculateContributingNumChan();
				if(nC == 6) {
					for(int i = 0; i < 3; i++) {
						in >> s->offset[i];
					}
					for(int i = 0; i < 3; i++) {
						in >> s->rot[i];
					}
				}
				else if (nC == 3) {
					for(int i = 0; i < 3; i++) {
						in >> s->rot[i];
					}
				}
				else if(nC != 0) {
					cout << "Unrecognised (" << nC << " != 6 or 3 or 0) number of channels for joint: " << s->getLongName() << endl;
				}
			}

			ss->update();
		}
		frames.push_back(frame);
	}
}

vector<Skeleton*> MotionData::cloneBaseSkeletons() {
	vector<Skeleton*> frame = baseSkeletons;
	for(Skeleton * & s : frame) { s = s->clone(); }
	return frame;
}

void MotionData::parseHierarchy(ifstream & in) {
	string word;
	nextWord(in); // ROOT
	parseRoot(in);
}

// Same as skeleton, but this is the top level
void MotionData::parseRoot(ifstream & in) {
	Skeleton * s = parseSkeleton(in);
	s->parent = NULL;
	baseSkeletons.push_back(s);
}

/**
 * Note that the fist keyword, (ROOT|JOINT) has already been read so we are left with:
 *
 * 			name {
 * 				OFFSET dx3			// in form X Y Z
 * 				CHANNELS i dxi		// assume XYZZXY (pos + rot) or just ZXY (rot)
 * 				JOINT subSkeleton   // 0 or many sub joints
 * 			}
 *
 * technically the JOINT keyword is part of the subskeleton, but it is parsed here
 */
Skeleton * MotionData::parseSkeleton(ifstream & in) {
	Skeleton * s = new Skeleton();

	// name
	s->name = nextWord(in);

	// {
	nextWord(in);

	// offset
	nextWord(in); // OFFSET
	in >> s->offset[0] >> s->offset[1] >> s->offset[2];

	// channels
	nextWord(in); // CHANNELS
	int numChan;
	in >> numChan;
	if(numChan == 3) { s->hasPosChan = false; }
	else if(numChan == 6) { s->hasPosChan = true; }
	else { cout << "BVHParser: Warning!!!! unexpected number of channels: " + numChan; }
	for(int i = 0; i < numChan; i++) { nextWord(in); }

	// Joints
	for(string word = nextWord(in); word != "}"; word = nextWord(in)) {
		Skeleton * sChild;
		if(word == "JOINT") {
			sChild = parseSkeleton(in);
		}
		else { // word == "End"
			sChild = new Skeleton();
			sChild->name = "End_Site";
			sChild->hasPosChan = false;
			nextWord(in); nextWord(in); nextWord(in); // Site { OFFSET
			in >> sChild->offset[0] >> sChild->offset[1] >> sChild->offset[2]; // get the offset
			nextWord(in); // }
		}
		sChild->parent = s;
		s->children.push_back(sChild);
	}

	// do the scaling/translatioin
	s->calculateScaleAndTranslate();

	// note that the final '}' is read in the wile loop above
	return s;

}
