#ifndef	_Util_H
#define	_Util_H	1

#include <functional>
#include <vector>
#include <Ogre.h>
#include <ode/ode.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

typedef Quaterniond Quat;
typedef Vector3d Vec3;
typedef Matrix3d Matrix3;


inline void print(Vec3 v) {
	cout << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")\n";
}

// degree to radian
inline double D2R(const double & x) {
	return 0.0174532925 * x;
}

inline Ogre::Vector3 ogreConv(Vector3d v) {
	return Ogre::Vector3((float) v[0],(float) v[1],(float) v[2]);
}

inline Ogre::Quaternion ogreConv(Quaterniond q) {
	return Ogre::Quaternion((Ogre::Real) q.w(),(Ogre::Real) q.x(),(Ogre::Real) q.y(),(Ogre::Real) q.z());
}

inline void dConv(Vector3d v, dVector3 out) {
	for(int i = 0; i < 3; i++) {
		out[i] = (dReal) v[i];
	}
}

inline void dConv(Quaterniond q, dQuaternion out) {
	out[0] = (dReal) q.w();
	out[1] = (dReal) q.x();
	out[2] = (dReal) q.y();
	out[3] = (dReal) q.z();
}

inline Vec3 eigVec3(const dVector3 v) {
	return Vec3(v[0], v[1], v[2]);
}

inline Quat eigQuat(const dQuaternion q) {
	return Quat(q[0], q[1], q[2], q[3]);
}

inline Matrix3d eigM3(const dMatrix3 md) {
	Matrix3d me;
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			me(i,j) = md[(3*i)+j];
		}
	}
	return me;
}

inline Matrix3 QuatToMatrix(Quat q) {

	Matrix3 m;

	m.col(0) = q * Vec3(1,0,0);
	m.col(1) = q * Vec3(0,1,0);
	m.col(2) = q * Vec3(0,0,1);

	return m;

}

inline Vec3 getAngularVelocity(Quat qi, Quat qf, double dt) {
	Quat qlnTerm = qf * qi.inverse();
	const float angleHalf = acos(qlnTerm.w());
	Vec3 rotAxis = qlnTerm.vec() / sin(angleHalf);
	return (2 * angleHalf * rotAxis) / dt;
}

/**
 * return a quaternion to rotate the dirI axis to pass through the given point (a.k.a face the given direction)
 */
inline Quat dirToDirQuat(Vec3 dirI, Vec3 dir) {
	if (abs(dirI.dot(dir)) < 0.0000001) {
		return Quat{0,0,0,1};
	}
	Quat q;
	Vec3 iDir = dirI.normalized();
	Vec3 tDir = dir.normalized();
	Vec3 axis = iDir.cross(tDir).normalized();
	double angle = acos(iDir.dot(tDir));
	return (Quaterniond) AngleAxisd(angle, axis);
}

/**
 * return a quaternion to rotate the z axis to pass through the given point (a.k.a face the given direction)
 */
inline Quat xToDirQuat(Vec3 dir) {
	return dirToDirQuat((Vec3) Vec3::UnitX(), dir);
}
inline Quat zToDirQuat(Vec3 dir) {
	return dirToDirQuat((Vec3) Vec3::UnitZ(), dir);
}
inline Quat yToDirQuat(Vec3 dir) {
	return dirToDirQuat((Vec3) Vec3::UnitY(), dir);
}

template<typename E>
inline bool contains(std::vector<E> v, E el) {
	return std::find(v.begin(),v.end(),el) != v.end();
}

template<typename E>
inline int indexOf(std::vector<E> v, E el) {
	int i = 0;
	for(typename std::vector<E>::iterator it = v.begin(); it != v.end(); it++, i++) {
		if(*it == el) {
			return i;
		}
	}
	return -1;
}

template<typename E>
inline int indexOf(const E * a, int size, E el) {
	for(int i = 0; i < size; i++) {
		if(a[i] == el) {
			return i;
		}
	}
	return -1;
}

template<typename E, typename F>
inline void forall(vector<E> & v, F f) {
	for_each(v.begin(), v.end(), f);
}

template<typename E, typename T, typename F>
inline vector<T> mapf(vector<E> & v, F f) {
	vector<T> n;
	forall(v, [&](E & e){
		n.push_back(f(e));
	});
	return n;
}

template<typename E, typename F>
inline void mapI(vector<E> & v, F f) {
	forall(v, [&](E & e){
		e = f(e);
	});
}


#endif
