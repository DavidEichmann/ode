#ifndef	_Util_H
#define	_Util_H	1

#include <Ogre.h>
#include <ode/ode.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// degree to radian
inline double D2R(const double & x) {
	return 0.0174532925 * x;
}

inline Ogre::Vector3 toVec3(Vector3d v) {
	return Ogre::Vector3((float) v[0],(float) v[1],(float) v[2]);
}

inline Ogre::Quaternion toQuat(Quaterniond q) {
	return Ogre::Quaternion((Ogre::Real) q.w(),(Ogre::Real) q.x(),(Ogre::Real) q.y(),(Ogre::Real) q.z());
}

inline void toDQuat(Quaterniond q, dQuaternion out) {
	out[0] = q.w();
	out[1] = q.x();
	out[2] = q.y();
	out[3] = q.z();
}

#endif
