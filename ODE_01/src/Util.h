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
