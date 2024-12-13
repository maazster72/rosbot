/*
 * Geoid.h
 *
 *  Created on: 11 Nov 2022
 *      Author: anon
 */

#ifndef SRC_GEOMETRY_GEOID_H_
#define SRC_GEOMETRY_GEOID_H_

#include "geometry/Coordinates.hpp"

namespace geometry {

class Geoid {

public:

	static double getOffset(const Point& point);

};

} /* namespace geometry */

#endif /* SRC_GEOMETRY_GEOID_H_ */
