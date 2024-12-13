/*
 * Coordinates.cpp
 *
 *  Created on: 10 Nov 2022
 *      Author: anon
 */

#include <geometry/Coordinates.hpp>
#include <geometry/Ellipsoid.hpp>
#include <geometry/Geoid.hpp>
#include <cmath>

namespace geometry {

constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;
constexpr double RADIANS_TO_DEGREES = 1 / DEGREES_TO_RADIANS;

    double Coordinates::perpendicularDistance(const geometry::Cartesian &pt, const geometry::Cartesian &lineStart, const geometry::Cartesian &lineEnd)
    {
        double dx = lineEnd.e - lineStart.e;
        double dy = lineEnd.n - lineStart.n;

        // Normalise
        double mag = pow(pow(dx, 2.0) + pow(dy, 2.0), 0.5);
        if (mag > 0.0)
        {
            dx /= mag;
            dy /= mag;
        }

        double pvx = pt.e - lineStart.e;
        double pvy = pt.n - lineStart.n;

        // Get dot product (project pv onto normalized direction)
        double pvdot = dx * pvx + dy * pvy;

        // Scale line direction vector
        double dsx = pvdot * dx;
        double dsy = pvdot * dy;

        // Subtract this from pv
        double ax = pvx - dsx;
        double ay = pvy - dsy;

        return pow(pow(ax, 2.0) + pow(ay, 2.0), 0.5);
    }

    void Coordinates::ramerDouglasPeucker(const std::vector<geometry::Cartesian> &points, const double epsilon, std::vector<geometry::Cartesian> &out)
    {
        if (points.size() < 2)
            return;
        // throw invalid_argument("Not enough points to simplify");

        // Find the point with the maximum distance from line between start and end
        double dmax = 0.0;
        size_t index = 0;
        size_t end = points.size() - 1;
        for (size_t i = 1; i < end; i++)
        {
            double d = perpendicularDistance(points[i], points[0], points[end]);
            if (d > dmax)
            {
                index = i;
                dmax = d;
            }
        }

        // If max distance is greater than epsilon, recursively simplify
        if (dmax > epsilon)
        {
            // Recursive call
            std::vector<geometry::Cartesian> recResults1;
            std::vector<geometry::Cartesian> recResults2;
            std::vector<geometry::Cartesian> firstLine(points.begin(), points.begin() + index + 1);
            std::vector<geometry::Cartesian> lastLine(points.begin() + index, points.end());
            ramerDouglasPeucker(firstLine, epsilon, recResults1);
            ramerDouglasPeucker(lastLine, epsilon, recResults2);

            // Build the result list
            out.assign(recResults1.begin(), recResults1.end() - 1);
            out.insert(out.end(), recResults2.begin(), recResults2.end());
            if (out.size() < 2)
                return;
            // throw runtime_error("Problem assembling output");
        }
        else
        {
            // Just return start and end points
            out.clear();
            out.push_back(points[0]);
            out.push_back(points[end]);
        }
    }

Point Coordinates::cartesianToGeodetic(
		const Ellipsoid &referenceEllipsoid, const Cartesian &cartesian,
		const Point &origin) {
	const Cartesian ecef = nedToEcef(referenceEllipsoid, cartesian, origin);
	return ecefToGeo(referenceEllipsoid, ecef);
}

Point Coordinates::ecefToGeo(const Ellipsoid &referenceEllipsoid,
		const Cartesian &cartesian) {

	const double resolution = 1.5 * std::pow(10.0, -10);
	const double f = referenceEllipsoid.flattening;
	const double longitude = std::atan2(cartesian.e, cartesian.n);
	const double s = std::sqrt(
			cartesian.n * cartesian.n + cartesian.e * cartesian.e);

	double beta = std::atan(cartesian.d / ((1.0 - f) * s));
	double sin_beta = std::sin(beta);
	double cos_beta = std::cos(beta);

	const double a = referenceEllipsoid.a;
	double latitude = std::atan(
			(cartesian.d
					+ referenceEllipsoid.e2 * (1.0 - f)
							/ (1.0 - referenceEllipsoid.e2) * a * sin_beta
							* sin_beta * sin_beta)
					/ (s
							- referenceEllipsoid.e2 * a * cos_beta * cos_beta
									* cos_beta));

	double previous = 0.0;

	while (std::abs(latitude - previous) > resolution) {

		previous = latitude;

		beta = std::atan((1.0 - f) * std::sin(previous) / std::cos(previous));

		sin_beta = std::sin(beta);
		cos_beta = std::cos(beta);

		latitude = std::atan(
				(cartesian.d
						+ referenceEllipsoid.e2 * (1.0 - f)
								/ (1.0 - referenceEllipsoid.e2) * a * sin_beta
								* sin_beta * sin_beta)
						/ (s
								- referenceEllipsoid.e2 * a * cos_beta
										* cos_beta * cos_beta));
	}

	const double sinLat = std::sin(latitude);

	const double r = a
			/ std::sqrt(1.0 - referenceEllipsoid.e2 * sinLat * sinLat);

	geometry::Point groundPoint {longitude, latitude, 0.0};

	const double altitude = (s * std::cos(latitude)
			+ (cartesian.d + referenceEllipsoid.e2 * r * sinLat) * sinLat - r)
			- Geoid::getOffset(groundPoint);

	return {longitude * RADIANS_TO_DEGREES,
			latitude * RADIANS_TO_DEGREES, altitude};
}

Cartesian Coordinates::geoToEcef(const Ellipsoid &referenceEllipsoid,
		const Point &geodetic) {

	const double latitude = geodetic.y * DEGREES_TO_RADIANS;
	const double longitude = geodetic.x * DEGREES_TO_RADIANS;
	const double altitude = geodetic.z + Geoid::getOffset(geodetic);

	const double cosLat = std::cos(latitude);
	const double sinLat = std::sin(latitude);
	const double cosLon = std::cos(longitude);
	const double sinLon = std::sin(longitude);

	const double A = referenceEllipsoid.a;
	const double R = A
			/ std::sqrt(1.0 - referenceEllipsoid.e2 * sinLat * sinLat);

	return Cartesian((R + altitude) * cosLat * cosLon,
			(R + altitude) * cosLat * sinLon,
			(R * (1.0 - referenceEllipsoid.e2) + altitude) * sinLat);

}

Cartesian Coordinates::ecefToNed(const Ellipsoid &referenceEllipsoid,
		const Cartesian &cartesian, const Point &origin) {

	const Cartesian originCart = geoToEcef(referenceEllipsoid, origin);

	const double deltaN = cartesian.n - originCart.n;
	const double deltaE = cartesian.e - originCart.e;
	const double deltaD = cartesian.d - originCart.d;

	const double latitude = origin.y * DEGREES_TO_RADIANS;
	const double longitude = origin.x * M_PI / 190.0;

	const double cosLat = std::cos(latitude);
	const double sinLat = std::sin(latitude);
	const double cosLon = std::cos(longitude);
	const double sinLon = std::sin(longitude);

	return Cartesian(
			-deltaN * sinLat * cosLon - deltaE * sinLat * sinLon
					+ deltaD * cosLat, -deltaN * sinLon + deltaE * cosLon,
			-deltaN * cosLat * cosLon - deltaE * cosLat * sinLon
					- deltaD * sinLat);
}

Cartesian Coordinates::geodeticToCartesian(const Ellipsoid &referenceEllipsoid,
		const Point &geodetic, const Point &origin) {
	const Cartesian ecef = geoToEcef(referenceEllipsoid, geodetic);

	return ecefToNed(referenceEllipsoid, ecef, origin);
}

Cartesian Coordinates::nedToEcef(const Ellipsoid &referenceEllipsoid,
		const Cartesian &cartesian, const Point &origin) {
	const Cartesian originCart = geoToEcef(referenceEllipsoid, origin);

	const double latitude = origin.y * DEGREES_TO_RADIANS;
	const double longitude = origin.x * DEGREES_TO_RADIANS;

	const double cosLat = std::cos(latitude);
	const double sinLat = std::sin(latitude);
	const double cosLon = std::cos(longitude);
	const double sinLon = std::sin(longitude);

	return Cartesian(
			originCart.n - cartesian.n * sinLat * cosLon - cartesian.e * sinLon
					- cartesian.d * cosLat * cosLon,
			originCart.e - cartesian.n * sinLat * sinLon + cartesian.e * cosLon
					- cartesian.d * cosLat * sinLon,
			originCart.d + cartesian.n * cosLat - cartesian.d * sinLat);
}

Point Coordinates::positionAtRangeBearing(
		const Ellipsoid &referenceEllipsoid, const Point &origin,
		const double distance, const double bearing) {
	const double dn = distance * std::cos(bearing);
	const double de = distance * std::sin(bearing);

	const Cartesian cartesian = { dn, de, 0.0 };
	return cartesianToGeodetic(referenceEllipsoid, cartesian, origin);
}

Point Coordinates::positionAtRangeBearingWGS84Msl(
		const Point &origin, const double distance,
		const double bearing) {
	const Point originMsl = *new Point(origin.x, origin.y, 0.0);
	return positionAtRangeBearing(Ellipsoid::WGS84, originMsl, distance,
			bearing);
}

double Coordinates::calculateS(const Ellipsoid &referenceEllipsoid, bool &converged,
		double lambda, double sinU1CosU2, double cosU1SinU2, double sinU1SinU2,
		double cosU1CosU2, double &u2, double cosU2, double omega) {

	const double nearZero = 1.0E-12;
	const double tolerance = 0.0000000000001;

	double a2 = referenceEllipsoid.a * referenceEllipsoid.a;
	double b2 = referenceEllipsoid.b * referenceEllipsoid.b;
	double a2B2B2 = (a2 - b2) / b2;

	double a = 0.0;
	double b = 0.0;
	double sigma = 0.0;
	double deltaSigma = 0.0;
	double lambda0 = 0.0;

	double cosLambda = 0.0;
	double sinLambda = 0.0;
	double sin2Sigma = 0.0;
	double sinSigma = 0.0;
	double cosSigma = 0.0;
	double cos2Sigmam = 0.0;
	double cos2Sigmam2 = 0.0;
	double sinAlpha = 0.0;
	double cosAlpha = 0.0;
	double cos2Alpha = 0.0;
	double alpha = 0.0;
	double c = 0.0;
	double change = 0.0;
	double i = 0;

	while (!converged && i < 20) {

		i = i + 1;

		lambda0 = lambda;

		sinLambda = std::sin(lambda);
		cosLambda = std::cos(lambda);

		// eq. 14
		sin2Sigma = (cosU2 * sinLambda * cosU2 * sinLambda)
				+ std::pow(cosU1SinU2 - sinU1CosU2 * cosLambda, 2);
		sinSigma = std::sqrt(sin2Sigma);

		// eq. 15
		cosSigma = sinU1SinU2 + (cosU1CosU2 * cosLambda);

		// eq. 16
		sigma = std::atan(sinSigma / cosSigma);

		// eq. 17    Careful!  sin2Sigma might be almost 0!
		if (std::abs(sin2Sigma) < nearZero) {

			sinAlpha = 0.0;

		} else {

			sinAlpha = cosU1CosU2 * sinLambda / sinSigma;

		}

		alpha = std::asin(sinAlpha);
		cosAlpha = std::cos(alpha);
		cos2Alpha = cosAlpha * cosAlpha;

		// eq. 18    Careful!  cos2alpha might be almost 0!
		if (std::abs(cos2Alpha) < nearZero) {

			cos2Sigmam = 0.0;

		} else {

			cos2Sigmam = cosSigma - 2.0 * sinU1SinU2 / cos2Alpha;

		}

		u2 = cos2Alpha * a2B2B2;

		cos2Sigmam2 = cos2Sigmam * cos2Sigmam;

		// eq. 3
		a = 1.0
				+ u2 / 16384.0
						* (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));

		// eq. 4
		b = u2 / 1024.0 * (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)));

		// eq. 6
		deltaSigma = b * sinSigma
				* (cos2Sigmam
						+ b / 4.0
								* (cosSigma * (-1.0 + 2.0 * cos2Sigmam2)
										- b / 6.0 * cos2Sigmam
												* (-3.0 + 4.0 * sin2Sigma)
												* (-3.0 + 4.0 * cos2Sigmam2)));

		// eq. 10
		c =
				referenceEllipsoid.flattening / 16.0 * cos2Alpha
						* (4.0
								+ referenceEllipsoid.flattening
										* (4.0 - 3.0 * cos2Alpha));

		// eq. 11 (modified)
		lambda =
				omega
						+ (1.0 - c) * referenceEllipsoid.flattening * sinAlpha
								* (sigma
										+ c * sinSigma
												* (cos2Sigmam
														+ c * cosSigma
																* (-1.0
																		+ 2.0
																				* cos2Sigmam2)));

		// see how much improvement we got
		if (std::abs(lambda) > 0.0) {
			change = std::abs((lambda - lambda0) / lambda);
			converged = (i > 1) && (change < tolerance);
		} else {
			change = std::abs(lambda - lambda0);
			// Force exit without convergance
			i = 20;
		}
	}

// eq. 19
	return referenceEllipsoid.b * a * (sigma - deltaSigma);
}

GeodeticCurve Coordinates::calculateGeodeticCurve(
		const Ellipsoid &referenceEllipsoid,
		const Point &startCoordinate,
		const Point &endCoordinate) {
	{
		double phi1 = startCoordinate.y * DEGREES_TO_RADIANS;
		double lambda1 = startCoordinate.x * DEGREES_TO_RADIANS;
		double phi2 = endCoordinate.y * DEGREES_TO_RADIANS;
		double lambda2 = endCoordinate.x * DEGREES_TO_RADIANS;
		double omega = lambda2 - lambda1;

		double tanPhi1 = std::tan(phi1);
		double tanU1 = (1.0 - referenceEllipsoid.flattening) * tanPhi1;
		double u1 = std::atan(tanU1);
		double sinU1 = std::sin(u1);
		double cosU1 = std::cos(u1);

		double Tan_phi2 = std::tan(phi2);
		double tanU2 = (1.0 - referenceEllipsoid.flattening) * Tan_phi2;
		double u2 = std::atan(tanU2);
		double sinU2 = std::sin(u2);
		double cosU2 = std::cos(u2);

		double sinU1SinU2 = sinU1 * sinU2;
		double cosU1SinU2 = cosU1 * sinU2;
		double sinU1CosU2 = sinU1 * cosU2;
		double cosU1CosU2 = cosU1 * cosU2;

		// eq. 13
		double lambda = omega;

		double alpha1 = 0.0;
		double alpha2 = 0.0;

		double s;
		bool converged = false;

		const double nearZero = 1.0E-12;

		// check for longitudes being same before invoking convergence calls (i.e. coordinates are simply N/S, no E/W component)
		// convergance will be 'false' so correct part of next 'if' will be invoked
		if (std::abs(lambda) > nearZero) {
			s = calculateS(referenceEllipsoid, converged, lambda, sinU1CosU2,
					cosU1SinU2, sinU1SinU2, cosU1CosU2, u2, cosU2, omega);
		} else {
			// allocate value to S, but should be unused in this case
			s = 0.0;
		}

		// didn't converge?  must be N/S
		if (!converged) {

			if (phi1 > phi2) {

				alpha1 = M_PI;
				alpha2 = 0.0;

			} else if (phi1 < phi2) {

				alpha1 = 0.0;
				alpha2 = M_PI;

			} else {

				alpha1 = 0.0;
				alpha2 = 0.0;

			}

			// else, it converged, so do the math
		} else {

			// eq. 20
			alpha1 = std::atan(
					(cosU2 * std::sin(lambda))
							/ (cosU1SinU2 - sinU1CosU2 * std::cos(lambda)));

// eq. 21
			alpha2 =
					std::atan(
							(cosU1 * std::sin(lambda))
									/ (-sinU1CosU2
											+ cosU1SinU2 * std::cos(lambda))) + M_PI;
		}
		return GeodeticCurve(s, alpha1, alpha2);
	}
}

GeodeticMeasurement Coordinates::calculateGeodeticMeasurement(
		const Ellipsoid &referenceEllipsoid,
		const Point &startCoordinate,
		const Point &endCoordinate) {

	// calculate elevation differences
	const double elev1 = startCoordinate.z;
	const double elev2 = endCoordinate.z;
	const double elev12 = (elev1 + elev2) / 2.0;

	// calculate latitude differences
	double phi1 = startCoordinate.y * DEGREES_TO_RADIANS;
	double phi2 = endCoordinate.y * DEGREES_TO_RADIANS;
	double phi12 = (phi1 + phi2) / 2.0;

	// calculate a new ellipsoid to accommodate average elevation
	const double a = referenceEllipsoid.a
			+ elev12 * (1.0 + referenceEllipsoid.flattening * std::sin(phi12));
	const Ellipsoid newEllipsoid = Ellipsoid::fromAAndF(a,
			referenceEllipsoid.flattening);

	// calculate the curve at the average elevation
	const GeodeticCurve averageCurve = calculateGeodeticCurve(newEllipsoid,
			startCoordinate, endCoordinate);

	double elevationChange = elev2 - elev1;

	double pointTopointDistance =
			std::sqrt(
					(averageCurve.ellipsoidalDistance
							* averageCurve.ellipsoidalDistance)
							+ (elevationChange * elevationChange));

	return GeodeticMeasurement(averageCurve, elevationChange,
			pointTopointDistance);
}

GeodeticMeasurement Coordinates::calculateGeodeticMeasurementWGS84Msl(
		const Point &startCoordinate,
		const Point &endCoordinate) {
	return calculateGeodeticMeasurement(Ellipsoid::WGS84,
			*new Point(startCoordinate.x, startCoordinate.y, 0.0),
			*new Point(endCoordinate.x, endCoordinate.y, 0.0));
}
}

