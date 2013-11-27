#include <string.h>
#include <iostream>
#include <fstream>

#include "BVHParser.h"
#include "Skeleton.h"

using namespace std;

BVHParser::BVHParser(const char * filePath) {
	ifstream in;
	in.open(filePath, ios::in);
	parse(in);
}

string BVHParser::nextWord(ifstream & in) {
	// read next word
	in >> wordBuf;
	return string(wordBuf);
}

void BVHParser::fillChannelsArray() {
	channels = new double*[numChan];
	double * * nextCPos = channels;
	if(skeletons.empty()) {
		clog << "BVHParser: Warning: no skeletons detected." << endl;
	}
	else {
		for(vector<Skeleton *>::iterator it = skeletons.begin(); it != skeletons.end(); it++) {
			fillChannelsArray(*it, nextCPos);
		}
	}
}

// traverse in the order of Root then Left-to-Right children
void BVHParser::fillChannelsArray(Skeleton * s, double * * nextCPos) {
	// either XYZZXY (pos + rot) or just ZXY (rot)
	int nC = s->calculateContributingNumChan();
	int i = nC;
	while(i > 0) {
		i--;
		nextCPos[i] = s->pos + i;
	}
	nextCPos += nC;

	// recurse to children
	if( ! s->children.empty()) {
		for(vector<Skeleton *>::iterator it = s->children.begin(); it != s->children.end(); it++) {
			fillChannelsArray(*it, nextCPos);
		}
	}
}

void BVHParser::calculateNumChan() {
	numChan = 0;
	if( ! skeletons.empty()) {
		for(vector<Skeleton*>::iterator itt = skeletons.begin(); itt != skeletons.end(); ++itt) {
			numChan += (*itt)->calculateNumChan();
		}
	}
}

void BVHParser::loadKeyframe(int index) {
	for(int c = 0; c < numChan; c++) {
		*(channels[c]) = keyframes[index][c];
	}
}

void BVHParser::parse(ifstream & in) {
	// heirarchy
	nextWord(in); // HEIRARCHY
	parseHierarchy(in);

	// count channels
	calculateNumChan();

	// fill the chanels array
	fillChannelsArray();

	// parse the keyframes
	parseKeyfames(in);
}

void BVHParser::parseKeyfames(ifstream & in) {
	double dt;	// frame duration

	// MOTION
	nextWord(in);

	// Frames:	765
	nextWord(in);
	in >> numFrames;
	keyframes = new double*[numFrames];

	// Frame Time:	0.0333333
	nextWord(in); nextWord(in);
	in >> dt;

	// keep reading values adding them circularly to channels
	// something like:  loop(i)  { in >> (*channels[i%numChan]) }
	for(int f = 0; f < numFrames; f++) {
		keyframes[f] = new double[numChan];
		for(int c = 0; c < numChan; c++) {
			in >> keyframes[f][c];
		}
	}
}

void BVHParser::parseHierarchy(ifstream & in) {
	string word;
	while( ! in.eof()) {
		word = nextWord(in);
		// parse accordingly
		if(word == "ROOT") {
			parseRoot(in);
		}
	}
}

// Same as skeleton, but this is the top level
void BVHParser::parseRoot(ifstream & in) {
	Skeleton * s = parseSkeleton(in);
	s->parent = NULL;
	skeletons.push_back(s);
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
Skeleton * BVHParser::parseSkeleton(ifstream & in) {
	Skeleton * s = new Skeleton();

	// name
	s->name = nextWord(in);

	// {
	nextWord(in);

	// offset
	nextWord(in); // OFFSET
	in >> s->pos[0] >> s->pos[1] >> s->pos[2];

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
			in >> sChild->pos[0] >> sChild->pos[1] >> sChild->pos[2]; // get the offset
			nextWord(in); // }
		}
		sChild->parent = s;
		s->children.push_back(sChild);
	}
	// note that the final '}' is read in the wile loop above
	return s;

}
