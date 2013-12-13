
#ifndef	_HUMAN_H
#define	_HUMAN_H	1

#include <map>
#include <ode/ode.h>

#include "Util.h"

using namespace std;

class Human {

public:

//	enum SegProps {
//		heightP,
//		widthP,
//		depthP,
//		massP,
//		parentSeg,
//		jointType,
//		SegProps_Length
//	};
//
//	enum SegNames {
//		head,
//		neck,
//		thorax,
//		abdomen,
//		pelvis,
//		upperarm,
//		forarm,
//		palm,
//		fingers,
//		thigh,
//		lowerleg,
//		foot,
//		SegNames_Length
//	};

	struct Props {
		string name;
		double height;
		double width;
		double depth;
		double mass;
		Vec3 pos;
		int parentIndex;
	};

	Human(char * model);
	~Human();
	void realize(dWorldID wid, dSpaceID sid);

	int getSize();
	double getHeight();
	const dBodyID * getBodyIDs();
	const Props * getProps();

private:

	static const int size = 19;	// number of segments
	double height;			// in m
	double mass;			// in kg
	Props props[size];		// properties of each segment
	dBodyID bodyIDs[size];
	map<string,string> parentNameMap;
	map<string,string> parentConnectMap;

	char wordBuf[100];

	string nextWord(ifstream & in);
	void parse(ifstream & in);
	void parseSegments(ifstream & in);
	void realizePositions();
	double realizePositionsRecMinY(int i);
	void translateYPos(double dy);
	const int getIndexByName(string name);
};

#endif
