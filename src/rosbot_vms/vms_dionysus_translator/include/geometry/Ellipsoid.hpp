/*
 * Ellipsoid.h
 *
 *  Created on: 10 Nov 2022
 *      Author: anon
 */

#ifndef SRC_ELLIPSOID_H_
#define SRC_ELLIPSOID_H_

namespace geometry {

class Ellipsoid {

public:

    double a; // Semi major axis
    double b; // Semi minor axis
    double flattening;
    double inverseFlattening;
    double e2; // The square of the first eccentricity.
    double ep2; // The square of the second eccentricity.
    double eccentricity;

    Ellipsoid(double a_in, double b_in,
            double flattening_in, double inverseFlattening_in);

    static Ellipsoid fromAAndF(double a, double f);

    static Ellipsoid fromAAndInverseF(double a, double inverse_f);

    static const Ellipsoid WGS84;

};

}

#endif /* SRC_ELLIPSOID_H_ */
