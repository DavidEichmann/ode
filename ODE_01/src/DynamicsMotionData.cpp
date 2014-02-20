
#include <string.h>
#include <vector>

#include "MotionData.h"
#include "DynamicsMotionData.h"

void DynamicsMotionData::parse(const char * filePath) {
	MotionData::parse(filePath);
	calculateDynamics();
}


void DynamicsMotionData::calculateDynamics() {
	for(auto p0 = frames.begin()+1; p0 != frames.end(); p0++) {
		// get a windows of 3 frames
		vector<Skeleton*> & f0 = *(p0-1);
		vector<Skeleton*> & f1 = *p0;

		// for each skeleton
		for(int i = 0; i < f0.size(); i++) {
			// get the skeleton for the current frames
			vector<Skeleton*> ss0 = f0[i]->getAllSkeletons();
			vector<Skeleton*> ss1 = f1[i]->getAllSkeletons();

			for(int j = 0; j < ss0.size(); j++) {
				Skeleton* s0 = ss0[j];
				Skeleton* s1 = ss1[j];

				// use frame	0 and 1		to calculate velocities
				Vec3 linVel = (s1->getPosComG() - s0->getPosComG()) / getFrameTime();
				Vec3 angVel = getAngularVelocity(s0->getRotG(), s1->getRotG(), getFrameTime());
				s0->setLinearVel( linVel );
				s1->setLinearVel( linVel );
				s1->setAngularVel( angVel );
				s0->setAngularVel( angVel );

				// use frame	0 and 1		to calculate accelerlkations
				Vec3 linAcc = (s1->getLinearVel() - s0->getLinearVel()) / getFrameTime();
				Vec3 angAcc = (s1->getAngularVel() - s0->getAngularVel()) / getFrameTime();
				s0->setLinearAcc( linAcc );
				s0->setAngularAcc( angAcc );
				s1->setLinearAcc( linAcc );
				s1->setAngularAcc( angAcc );
			}
		}
	}
}
