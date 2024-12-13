/*
 * Coordinates.h
 *
 *  Created on: 10 Nov 2022
 *      Author: anon
 */

#ifndef SRC_GEOMETRY_COORDINATES_H_
#define SRC_GEOMETRY_COORDINATES_H_

#include <geometry/Ellipsoid.hpp>
#include <vector>

namespace geometry {

struct Cartesian {
	double n;
	double e;
	double d;

	Cartesian(double n_in, double e_in, double d_in) :
			n(n_in), e(e_in), d(d_in) {
	}
	;
};

struct Point {
	double x;
	double y;
	double z;

	Point(double x_in, double y_in, double z_in) :
			x(x_in), y(y_in), z(z_in) {
	}
	;
};

struct GeodeticCurve {
	double ellipsoidalDistance;
	double a1;
	double a2;
	GeodeticCurve(double ellipsoidal_distance_in, double a1_in, double a2_in) : ellipsoidalDistance(ellipsoidal_distance_in), a1(a1_in), a2(a2_in) {};
};

struct GeodeticMeasurement {
	GeodeticCurve curve;
	double elevationChange;
	double pointToPointDistance;

	GeodeticMeasurement(GeodeticCurve curve_in, double elevationChange_in, double pointToPointDistance_in) : curve(curve_in), elevationChange(elevationChange_in), pointToPointDistance(pointToPointDistance_in) {};
};

class Coordinates {

public:

	static Point cartesianToGeodetic(const Ellipsoid &referenceEllipsoid,
			const Cartesian &cartesian, const Point &origin);

	static Point ecefToGeo(const Ellipsoid& referenceEllipsoid,
			const Cartesian &cartesian);

	static Cartesian geoToEcef(const Ellipsoid &referenceEllipsoid,
			const Point &geodetic);

	static Cartesian geodeticToCartesian(const Ellipsoid &referenceEllipsoid,
			const Point &geodetic, const Point &origin);

	static Cartesian ecefToNed(const Ellipsoid &referenceEllipsoid,
			const Cartesian &cartesian, const Point &origin);

	static Cartesian nedToEcef(const Ellipsoid &referenceEllipsoid,
			const Cartesian &cartesian, const Point &origin);

	static Point positionAtRangeBearing(
			const Ellipsoid &referenceEllipsoid,const Point &origin,
			const double distance, const double bearing);

	static Point positionAtRangeBearingWGS84Msl(const Point& Origin,
			const double distance, const double bearing);

	static GeodeticMeasurement calculateGeodeticMeasurement(
			const Ellipsoid &referenceEllipsoid, const Point &startCoordinate,
			const Point &endCoordinate);

	static GeodeticMeasurement calculateGeodeticMeasurementWGS84Msl(
			const Point &startCoordinate, const Point &endCoordinate);

	static GeodeticCurve calculateGeodeticCurve(
			const Ellipsoid &referenceEllipsoid, const Point &startCoordinate,
			const Point &endCoordinate);

	static void ramerDouglasPeucker(const std::vector<geometry::Cartesian> &points, const double epsilon, std::vector<geometry::Cartesian> &out);

private:

    static double perpendicularDistance(const geometry::Cartesian &pt, const geometry::Cartesian &lineStart, const geometry::Cartesian &lineEnd);

	static double calculateS(const Ellipsoid &referenceEllipsoid, bool &converged,
	        double lambda, double sinU1CosU2, double cosU1SinU2, double sinU1SinU2,
	        double cosU1CosU2, double &u2, double cosU2, double omega);

};

}

#endif /* SRC_GEOMETRY_COORDINATES_H_ */
