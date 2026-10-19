#pragma once

#include <optional>
#include <string>

#include "demdl/BoundingBox.h"
#include "mesh/GeoTiffGrid.h"

namespace demdl::mesh {

// The window of samples that a crop box asks for: the smallest range of whole columns and rows whose first samples lie
// at or beyond the box's west and north edges and whose last samples lie at or beyond its east and south edges, clipped
// to the file. The mesh runs from the first sample to the last, so it covers the box wherever the file does. nullopt
// gives the whole file. geometry is what inspectGeoTiff returned, so dx and dy are positive.
//
// A Transverse Mercator file's box is projected first: its four corners and the midpoints of its four edges (spec
// 5.1), and the bounding rectangle of those eight points is cut as above. The window is then a little larger than the
// box. Eight points bound the box exactly when it is clear of the central meridian. A box across it can poke out of
// the window on its north edge (its south edge in the northern hemisphere), where a parallel's northing peaks at the
// meridian rather than at the box's midpoint: at 34 S, by about a metre for a box 18 km wide whose middle is 0.05
// degrees from the meridian, and by 28 m for one 140 km wide whose middle is 0.25 degrees from it.
//
// An edge within a millionth of a sample of a sample counts as on it. In doubles 18.2 and 0.01 are not exact, so an
// edge cut along sample 20 comes out as 19.99999999999993 samples, and plain floor and ceil would add a column or row
// of slack on one side at random and would not see that a box with both edges on one sample line has no extent.
//
// Throws std::runtime_error:
// - "the crop window overlaps the file by fewer than 2 x 2 samples", when clipping leaves fewer than two columns or two
//   rows: the box misses the file or only touches its edge (a mesh needs a cell);
// - "the file has fewer than 2 x 2 samples", for the same reason when there is no crop.
// Throws std::invalid_argument when an edge is not finite or the box is inside out (a north below its south or an east
// west of its west). BoundingBox::validate gives the user-facing messages for those, and callers check it first; this
// only keeps a NaN from reaching the conversion to int.
[[nodiscard]] PixelWindow pixelWindowFor(const GridGeometry& geometry, const std::optional<BoundingBox>& crop);

struct MetresPerDegree {
  double perDegreeLatitude = 0.0;
  double perDegreeLongitude = 0.0;
};

// The length on the ground of one degree of latitude and of one degree of longitude at a latitude in degrees, from the
// spec's formulas (5.2): 111,132.954 - 559.822 cos 2phi + 1.175 cos 4phi and 111,412.84 cos phi - 93.5 cos 3phi. They
// are within 0.2 m of the WGS 84 ellipsoid's own figures up to 70 degrees. A local scale for a relief model, not a
// projection.
[[nodiscard]] MetresPerDegree metresPerDegree(double latDeg) noexcept;

// The size of a window in metres. widthM and heightM run from the first sample to the last, which is how the mesh is
// laid out, so they are (cols - 1) and (rows - 1) cells; cellDxM and cellDyM are one cell across and down.
struct WindowExtent {
  double widthM = 0.0;
  double heightM = 0.0;
  double cellDxM = 0.0;
  double cellDyM = 0.0;
};

// A Transverse Mercator file's cells are dx by dy metres as they stand. A geographic file's degrees are turned into
// metres with metresPerDegree at the window's middle latitude, so the cells are the same size all over the window
// (within 0.1 percent over 10 km). window lies inside geometry's samples, as pixelWindowFor gives it.
[[nodiscard]] WindowExtent extentInMetres(const GridGeometry& geometry, const PixelWindow& window);

// One line for the dry run: the window's size and position in pixels, then where its first and last samples lie, in
// degrees for a geographic file and in projected metres for a Transverse Mercator one:
//   32 x 31 pixels at (20, 10), samples from longitude 18.2000000 to 18.5100000 and latitude -33.1000000 to
//   -33.4000000 degrees
// Latitudes run from the window's north edge to its south edge, as its rows do.
[[nodiscard]] std::string describeWindow(const GridGeometry& geometry, const PixelWindow& window);

}  // namespace demdl::mesh
