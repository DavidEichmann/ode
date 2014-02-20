
#ifndef	_DYNAMICSMOTIONDATA_H
#define	_DYNAMICSMOTIONDATA_H	1

#include <string.h>
#include <vector>

#include "MotionData.h"

/**
 * This class extends MotionData, adding dynamics information to the underlying keyframes
 */
class DynamicsMotionData : public MotionData {

public:

	DynamicsMotionData() : MotionData() {};
	DynamicsMotionData(const char * filePath) : MotionData(filePath) {};
	void parse(const char * filePath);

private:

	void calculateDynamics();

};


#endif
