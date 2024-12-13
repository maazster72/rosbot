/*
 * Ellipsoid.cpp
 *
 *  Created on: 10 Nov 2022
 *      Author: anon
 */

#include "geometry/Ellipsoid.hpp"
#include <cmath>

namespace geometry {

Ellipsoid::Ellipsoid(double a_in, double b_in, double flattening_in,
        double inverseFlattening_in) :
        a(a_in), b(b_in), flattening(flattening_in), inverseFlattening(
                inverseFlattening_in), e2(flattening * (2.0 - flattening)), ep2(((a * a - b * b)
                        / (b * b))), eccentricity(std::sqrt(e2)) {
}

Ellipsoid Ellipsoid::fromAAndInverseF(double semiMajor, double inverseFlattening) {
    const double flattening = 1.0 / inverseFlattening;

    return Ellipsoid(semiMajor, (1.0 - flattening) * semiMajor, flattening,
            inverseFlattening);
}

Ellipsoid Ellipsoid::fromAAndF(double semiMajor, double flattening) {
    double inverseFlattening;

    if (flattening > 0.0 || flattening < 0.0) {
        inverseFlattening = 1.0 / flattening;
    } else {
        inverseFlattening =flattening;
    }

    return Ellipsoid(semiMajor, (1.0 - flattening) * semiMajor, flattening,
            inverseFlattening);
}

const Ellipsoid Ellipsoid::WGS84 = fromAAndInverseF(6378137.0, 298.257223563);

}
