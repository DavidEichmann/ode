#include <ode/ode.h>
#include <iostream>
#include <fstream>
#include <queue>

#include "Constants.h"
#include "Human.h"

using namespace std;


Human::Human(char* model) {
	// init class variables
	parentNameMap["head"] 		= "neck";
	parentNameMap["neck"] 		= "thorax";
	parentNameMap["thorax"] 	= "abdomen";
	parentNameMap["abdomen"] 	= "pelvis";
	parentNameMap["pelvis"] 	= "ROOT";

	parentNameMap["upperarml"] 	= "thorax";
	parentNameMap["forarml"] 	= "upperarml";
	parentNameMap["palml"] 		= "forarml";
	parentNameMap["fingersl"] 	= "palml";
	parentNameMap["thighl"] 	= "pelvis";
	parentNameMap["lowerlegl"] 	= "thighl";
	parentNameMap["footl"] 		= "lowerlegl";

	parentNameMap["upperarmr"] 	= "thorax";
	parentNameMap["forarmr"] 	= "upperarmr";
	parentNameMap["palmr"] 		= "forarmr";
	parentNameMap["fingersr"] 	= "palmr";
	parentNameMap["thighr"] 	= "pelvis";
	parentNameMap["lowerlegr"] 	= "thighr";
	parentNameMap["footr"] 		= "lowerlegr";


	parentConnectMap["head"] 		= "tccbcc";
	parentConnectMap["neck"] 		= "tccbcc";
	parentConnectMap["thorax"] 		= "tccbcc";
	parentConnectMap["abdomen"] 	= "tccbcc";
	parentConnectMap["pelvis"] 		= "ROOT";

	parentConnectMap["upperarml"] 	= "tlctrc";
	parentConnectMap["forarml"] 	= "bcctcc";
	parentConnectMap["palml"] 		= "bcctcc";
	parentConnectMap["fingersl"] 	= "bcctcc";
	parentConnectMap["thighl"] 		= "bLctcc";
	parentConnectMap["lowerlegl"] 	= "bcctcc";
	parentConnectMap["footl"] 		= "bcctcB";

	parentConnectMap["upperarmr"] 	= "trctlc";
	parentConnectMap["forarmr"] 	= "bcctcc";
	parentConnectMap["palmr"] 		= "bcctcc";
	parentConnectMap["fingersr"] 	= "bcctcc";
	parentConnectMap["thighr"] 		= "bRctcc";
	parentConnectMap["lowerlegr"] 	= "bcctcc";
	parentConnectMap["footr"] 		= "bcctcB";

	// start parsing
	ifstream in;
	in.open(model, ios::in);
	parse(in);
}

Human::~Human() {
}

string Human::nextWord(ifstream & in) {
	// read next word
	in >> wordBuf;
	string w = string(wordBuf);
	if(w == "/*") {
		while (w != "*/") {
			in >> wordBuf;
			w = string(wordBuf);
		}
		in >> wordBuf;
		w = string(wordBuf);
	}
	return w;
}

void Human::parse(ifstream & in) {
	nextWord(in); // HEIGHT
	in >> height;
	nextWord(in); // MASS
	in >> mass;
	nextWord(in); // SEGMENTS
	parseSegments(in);

	// complete the positioning of segments
	realizePositions();
}

int Human::getSize() {
	return size;
}

double Human::getHeight() {
	return height;
}
const int Human::getIndexByName(string name) {
	for(int i = 0; i < size; i++) {
		if(props[i].name == name) {
			return i;
		}
	}
	return -1; // index out of bounds
}

const Human::Props * Human::getProps() {
	return props;
}

const dBodyID * Human::getBodyIDs() {
	return bodyIDs;
}

void Human::parseSegments(ifstream & in) {

	nextWord(in); // {
	string name;

	// non-limb segments
	string nonlimbsA[] = {"head", "neck", "thorax", "abdomen", "pelvis"};
	vector<string> nonlimbs(nonlimbsA, nonlimbsA+5);

	// load all properties
	int i = 0;
	while((name = nextWord(in)) != "}") {
		Props p;
		p.name = name;
		in >> p.height >> p.width >> p.depth >> p.mass;
		double hp = height*0.01; // convert percentage points to actual height
		p.height *= hp;
		p.width *= hp;
		p.depth *= hp;
		p.mass 	*= mass * 0.01;
		if(contains(nonlimbs, name)) {
			props[i++] = p;
		}
		else {
			Props pl = p;
			pl.name += "l";
			props[i++] = pl;
			Props pr = p;
			pr.name += "r";
			props[i++] = pr;
		}
	}

	// calculate parent indexes
	for(int i = 0; i < size; i++) {
		props[i].parentIndex = getIndexByName(parentNameMap[props[i].name]);
	}
}

void Human::realize(dWorldID wid, dSpaceID sid) {
	// for each body segment
	for(int i = 0; i < size; i++) {
		// create body
		dBodyID bid = bodyIDs[i] = dBodyCreate(wid);

		// create geom
		Props p = props[i];
		dGeomID gid = dCreateBox(sid,p.width,p.height,p.depth);
		dGeomSetBody(gid,bid);

		// mass
		dMass * m = new dMass;
		dMassSetBoxTotal(m,p.mass,p.width,p.height,p.depth);
		dBodySetMass(bid, m);

		// position
		dBodySetPosition(bid,(dReal)p.pos[0],(dReal)p.pos[1],(dReal)p.pos[2]);
	}

	// create joints for each body segment
	dJointGroupID jgid = dJointGroupCreate(40);
	for(int i = 0; i < size; i++) {
		int parentIndex = props[i].parentIndex;
		if(parentIndex != -1) {
			Props parentProps = props[parentIndex];
			Vec3 cPos = getConnectPoint(parentProps,parentConnectMap[props[i].name]);
			dJointID jid;
			// setup a ball joint
			jid = dJointCreateBall(wid, jgid);
			dJointAttach(jid,bodyIDs[parentIndex],bodyIDs[i]);
			dJointSetBallAnchor(jid, (double) cPos[0], (double) cPos[1], (double) cPos[2]);

			// setup constraints with an AMotor
			dJointID amid = dJointCreateAMotor(wid,jgid);
			dJointAttach(amid,bodyIDs[i],bodyIDs[parentIndex]);
			dJointSetAMotorMode(amid,dAMotorEuler);
			dJointSetAMotorAxis(amid,0,1, 0,0,1);
			dJointSetAMotorAxis(amid,2,2, 0,1,0);

			//dJointSetAMotorParam(amid,dParamStopERP,0);

			dJointSetAMotorParam(amid,dParamLoStop1,-PI/20);
			dJointSetAMotorParam(amid,dParamHiStop1,PI/20);

			dJointSetAMotorParam(amid,dParamLoStop2,-PI/20);
			dJointSetAMotorParam(amid,dParamHiStop2,PI/20);

			dJointSetAMotorParam(amid,dParamLoStop3,-PI/20);
			dJointSetAMotorParam(amid,dParamHiStop3,PI/20);

			dJointSetAMotorParam(amid,dParamFMax1,0.1);
			dJointSetAMotorParam(amid,dParamVel1,0);
			dJointSetAMotorParam(amid,dParamFMax2,0.1);
			dJointSetAMotorParam(amid,dParamVel2,0);
			dJointSetAMotorParam(amid,dParamFMax3,0.1);
			dJointSetAMotorParam(amid,dParamVel3,0);
		}
	}
}


Vec3 Human::getConnectPoint(Props parent, string connectString) {

	Vec3 pos = parent.pos;

	if(connectString[0] == 'b') 		{ pos[1] -= parent.height / 2; }
	else if(connectString[0] == 't') 	{ pos[1] += parent.height / 2; }
	if(connectString[1] == 'l') 		{ pos[0] -= parent.width  / 2; }
	else if(connectString[1] == 'L') 	{ pos[0] -= parent.width  / 4; }
	else if(connectString[1] == 'r') 	{ pos[0] += parent.width  / 2; }
	else if(connectString[1] == 'R') 	{ pos[0] += parent.width  / 4; }
	if(connectString[2] == 'b') 		{ pos[2] -= parent.depth  / 2; }
	else if(connectString[2] == 'B') 	{ pos[2] -= parent.depth  / 4; }
	else if(connectString[2] == 'f') 	{ pos[2] += parent.depth  / 2; }
	else if(connectString[2] == 'F') 	{ pos[2] += parent.depth  / 4; }

	return pos;
}

void Human::realizePositions() {
	int rootI;
	for(rootI = 0; parentConnectMap[props[rootI].name] != "ROOT"; rootI++) {}
	double dy = -1 * realizePositionsRecMinY(rootI);
	translateYPos(dy);
}
double Human::realizePositionsRecMinY(int i) {
	Props& p = props[i];
	// set my pos
	string parentName = parentNameMap[p.name];
	if(parentName == "ROOT") {
		p.pos = Vec3(0,0,0);
	}
	else {
		// logic is Pos = ParentPos + (ParentPos->ParentConnectPoint) - (Pos->ConnectPoint)
		//  where A->B is a vector from point A to B. above both vectors can be calculated locally
		Props pp = props[getIndexByName(parentName)];
		string r = parentConnectMap[p.name];
		p.pos = getConnectPoint(pp, r);

		if(r[3] == 'b') 		{ p.pos[1] += p.height / 2; }
		else if(r[3] == 't') 	{ p.pos[1] -= p.height / 2; }
		if(r[4] == 'l') 		{ p.pos[0] += p.width  / 2; }
		else if(r[4] == 'L') 	{ p.pos[0] += p.width  / 4; }
		else if(r[4] == 'r') 	{ p.pos[0] -= p.width  / 2; }
		else if(r[4] == 'R') 	{ p.pos[0] -= p.width  / 4; }
		if(r[5] == 'b') 		{ p.pos[2] += p.depth  / 2; }
		else if(r[5] == 'B') 	{ p.pos[2] += p.depth  / 4; }
		else if(r[5] == 'f') 	{ p.pos[2] -= p.depth  / 2; }
		else if(r[5] == 'F') 	{ p.pos[2] -= p.depth  / 4; }
	}

	// recurs to all children
	double minY = p.pos[1] - (p.height/2);
	for(int j = 0; j < size; j++) {
		if(props[j].parentIndex == i) {
			minY = min(minY,realizePositionsRecMinY(j));
		}
	}
	return minY;
}
void Human::translateYPos(double dy) {
	for(int i = 0; i < size; i++) {
		props[i].pos[1] += dy;
	}
}
