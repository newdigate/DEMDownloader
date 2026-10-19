#pragma once

#include <optional>

namespace demdl::mesh {

// What a Transverse Mercator projection needs, as a projected GeoTIFF states it. Angles are degrees, lengths metres.
// The defaults are a scale of 1, no false origin and the GRS 80 ellipsoid, so setting the central meridian alone
// gives a South African Lo system such as Lo19.
struct TransverseMercatorParams {
  double originLongitudeDeg = 0.0;  // the central meridian
  double originLatitudeDeg = 0.0;   // latitude of the natural origin
  double scale = 1.0;               // on the central meridian
  double falseEasting = 0.0;        // the easting of the natural origin
  double falseNorthing = 0.0;       // the northing of the natural origin
  double semiMajorAxis = 6378137.0;
  // 1/f, GRS 80 by default. 0 or infinity means a sphere: WKT writes a sphere as 0, and a semi-minor axis equal to the
  // semi-major one (which is how a GeoTIFF can state a sphere) gives infinity.
  double inverseFlattening = 298.257222101;
};

struct Projected {
  double x = 0.0;  // easting, metres
  double y = 0.0;  // northing, metres
};

// Forward projection of a longitude and latitude in degrees, taken as geodetic coordinates on the params' ellipsoid;
// no datum shift is applied. A longitude may be written either way round the antimeridian (179.5 or -180.5), so UTM
// zones 1 and 60 work across it. A NaN or infinite coordinate gives NaN in both x and y.
//
// Krüger's series, fourth order in the third flattening n: no iteration and no trouble at the poles. Against PROJ 9.4
// it differs by under a micrometre within 20 degrees of the central meridian, by 2e-5 m at 40 degrees and by up to
// 7 mm at 60, far inside the millimetre the mesh tool needs within a few degrees. It is not meant for points much
// farther out: the projection is singular 90 degrees from the central meridian on the equator, where the result is
// huge but finite.
[[nodiscard]] Projected forwardTransverseMercator(const TransverseMercatorParams& params, double lonDeg,
                                                  double latDeg) noexcept;

// The parameters of a UTM zone from its WGS 84 EPSG code: 32601 to 32660 are zones 1 to 60 north and 32701 to 32760
// are zones 1 to 60 south. Zone z has its central meridian at -183 + 6 z degrees, scale 0.9996, false easting 500 km
// and a false northing of 0 in the north and 10,000 km in the south. Any other code gives nullopt, the polar UPS grids
// (32661, 32761) and the South African Lo systems among them: this is a small table, not an EPSG database.
[[nodiscard]] std::optional<TransverseMercatorParams> utmFromEpsg(int code);

}  // namespace demdl::mesh
