#include "mesh/TransverseMercator.h"

#include <cmath>
#include <numbers>

namespace demdl::mesh {

namespace {

constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;

// What Krüger's series needs from the ellipsoid. The series is in powers of the third flattening
// n = (a - b) / (a + b), about 1.7e-3 for the Earth, so each order gains three digits and the fifth, which is left
// out, is about 1e-14 of the radius.
struct Series {
  double eccentricity;
  double rectifyingRadius;  // metres: the radius of a sphere whose meridian is as long as the ellipsoid's
  double alpha[4];          // the forward coefficients, orders 1 to 4
};

Series seriesFor(double semiMajorAxis, double inverseFlattening) noexcept {
  // A sphere is written as an inverse flattening of 0, and 1/0 would turn n into NaN. Infinity, the other way to get
  // a sphere, already gives f = 0.
  const double f = inverseFlattening == 0.0 ? 0.0 : 1.0 / inverseFlattening;
  const double n = f / (2.0 - f);
  const double n2 = n * n;
  const double n3 = n2 * n;
  const double n4 = n2 * n2;
  Series s{};
  s.eccentricity = std::sqrt(f * (2.0 - f));
  s.rectifyingRadius = semiMajorAxis / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0);
  s.alpha[0] = n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0;
  s.alpha[1] = 13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0;
  s.alpha[2] = 61.0 * n3 / 240.0 - 103.0 * n4 / 140.0;
  s.alpha[3] = 49561.0 * n4 / 161280.0;
  return s;
}

// A position in the series' own units: xi is the northing and eta the easting, each divided by the rectifying radius
// (and by the scale).
struct Rectified {
  double xi;
  double eta;
};

// The position of latitude phi and longitude difference dLambda (from the central meridian), both in radians.
Rectified rectify(const Series& s, double phi, double dLambda) noexcept {
  // The ellipsoid maps conformally onto a sphere whose latitude chi follows from the isometric latitude psi; on that
  // sphere the projection has a closed form, and the series carries it back to the ellipsoid. tan(chi) is sinh(psi),
  // so chi itself is never needed. At a pole tan(phi) is huge but finite, so psi stays finite too.
  const double psi = std::asinh(std::tan(phi)) - s.eccentricity * std::atanh(s.eccentricity * std::sin(phi));
  const double tanChi = std::sinh(psi);
  // The sphere's transverse Mercator. atan2 keeps the quadrant beyond 90 degrees from the central meridian, and the
  // asinh form is atanh(cos(chi) * sin(dLambda)) without its infinity at 90 degrees.
  const double xiSphere = std::atan2(tanChi, std::cos(dLambda));
  const double etaSphere = std::asinh(std::sin(dLambda) / std::hypot(tanChi, std::cos(dLambda)));
  Rectified r{xiSphere, etaSphere};
  for (int order = 1; order <= 4; ++order) {
    const double alpha = s.alpha[order - 1];
    const double k = 2.0 * order;
    r.xi += alpha * std::sin(k * xiSphere) * std::cosh(k * etaSphere);
    r.eta += alpha * std::cos(k * xiSphere) * std::sinh(k * etaSphere);
  }
  return r;
}

}  // namespace

Projected forwardTransverseMercator(const TransverseMercatorParams& params, double lonDeg, double latDeg) noexcept {
  const Series s = seriesFor(params.semiMajorAxis, params.inverseFlattening);
  // The difference enters only through its sine and cosine, so 356.5 degrees (a point across the antimeridian from
  // the central meridian) needs no wrapping into [-180, 180] to give the same answer as -3.5.
  const double dLambda = (lonDeg - params.originLongitudeDeg) * kRadiansPerDegree;
  const Rectified at = rectify(s, latDeg * kRadiansPerDegree, dLambda);
  // The natural origin's northing is not 0 when its latitude is not, so it is taken off by running it through the
  // same series; the false easting and northing are added after the scale, as EPSG defines them.
  const Rectified origin = rectify(s, params.originLatitudeDeg * kRadiansPerDegree, 0.0);
  const double radius = params.scale * s.rectifyingRadius;
  return {params.falseEasting + radius * at.eta, params.falseNorthing + radius * (at.xi - origin.xi)};
}

std::optional<TransverseMercatorParams> utmFromEpsg(int code) {
  // EPSG:326zz is "WGS 84 / UTM zone zz N" and EPSG:327zz is zone zz S.
  const bool north = code >= 32601 && code <= 32660;
  const bool south = code >= 32701 && code <= 32760;
  if (!north && !south) return std::nullopt;
  const int zone = code - (north ? 32600 : 32700);

  TransverseMercatorParams p;
  p.originLongitudeDeg = -183.0 + 6.0 * zone;
  p.scale = 0.9996;
  p.falseEasting = 500000.0;
  p.falseNorthing = north ? 0.0 : 10000000.0;
  // WGS 84, whose inverse flattening is 1.5e-6 away from the struct's GRS 80 default.
  p.inverseFlattening = 298.257223563;
  return p;
}

}  // namespace demdl::mesh
