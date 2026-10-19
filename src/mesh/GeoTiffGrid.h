#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "mesh/TransverseMercator.h"

// Fast-math lets the compiler fold std::isnan to false, and then no-data would read as terrain.
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "NaN marks no data; do not build with -ffast-math or -ffinite-math-only"
#endif

namespace demdl::mesh {

enum class ModelKind { Geographic, TransverseMercator };

// Where the samples of a single-band elevation GeoTIFF are. x and y are longitude and latitude in degrees for a
// geographic file, easting and northing in metres for a Transverse Mercator one. Sample (col, row) sits at
// (sampleX(col), sampleY(row)): columns run east and rows south, so dx and dy are both positive.
struct GridGeometry {
  ModelKind kind = ModelKind::Geographic;
  // The projection of a Transverse Mercator file. For a geographic file only the ellipsoid is set from the file.
  TransverseMercatorParams tm;
  int width = 0, height = 0;  // samples across and down
  double sampleX0 = 0.0, sampleY0 = 0.0;  // the position of sample (0, 0)
  double dx = 0.0, dy = 0.0;              // from one column to the next, east; from one row to the next, south
  bool pixelIsPoint = false;  // RasterPixelIsPoint; otherwise RasterPixelIsArea, which is also what no key means
  bool dlElevation = false;   // the ImageDescription starts "dl_elevation;"
  std::optional<double> noData;  // GDAL_NODATA, when the file has it
  std::string description;       // the ImageDescription, empty when there is none

  [[nodiscard]] double sampleX(int col) const { return sampleX0 + col * dx; }
  [[nodiscard]] double sampleY(int row) const { return sampleY0 - row * dy; }
};

// Columns col0 to col0 + cols - 1 of rows row0 to row0 + rows - 1.
struct PixelWindow {
  int col0 = 0, row0 = 0, cols = 0, rows = 0;
};

// Reads where a GeoTIFF's samples are, from its tags and GeoKeys, without reading a sample.
//
// Accepted: one sample per pixel of float32, float64, int16, int32 or uint16, in strips or tiles; a ModelPixelScale
// with a ModelTiepoint at any raster position (the first, when there are several), or a ModelTransformation without
// rotation or shear; north up; a geographic model in degrees, or a projected one that is Transverse Mercator by its
// GeoKeys (ProjCoordTrans 1 with at least its central meridian) or by a UTM zone's EPSG code alone; lengths and
// heights in metres. Sample (0, 0) is raster position (0, 0) for RasterPixelIsPoint, and half a pixel east and south
// of it, at the centre of pixel (0, 0), for RasterPixelIsArea, except in a file dl_elevation wrote: the server it reads
// samples each pixel's north-west corner, so those samples stay on the corners.
//
// The ellipsoid comes from GeogSemiMajorAxis and GeogInvFlattening (or GeogSemiMinorAxis), each key overriding its own
// value, else from GeogEllipsoid 7019 (GRS 1980) or 7030 (WGS 84), else GRS 80. A UTM code gives WGS 84. Lengths
// and angles without a units GeoKey are taken as metres and degrees.
//
// Throws std::runtime_error, with the file's name in the message, when the file cannot be opened or is anything else:
// another sample type or band count; no tiepoint or transformation, or a tiepoint without a pixel scale; rotated,
// sheared, mirrored or upside down; placed by numbers that are not finite; neither geographic nor projected; another
// projection; south-oriented (CT_TransvMercator_SouthOriented, or one of EPSG's Lo grids 2046 to 2055, whose axes run
// west and south); an EPSG code without parameters that is not a UTM zone; Transverse Mercator parameters or an
// ellipsoid that are not usable numbers; lengths, heights or angles in other units; a GeoKey stored as another type
// than the standard's; or a GDAL_NODATA that is not a number. Messages that libtiff gives are appended.
[[nodiscard]] GridGeometry inspectGeoTiff(const std::filesystem::path& path);

// The window's samples as heights, row by row from its north-west corner (cols x rows values). The file's no-data
// value, NaN, and an infinite value, which no terrain has, all become NaN; float64 heights beyond float32's range do
// too. geometry is what inspectGeoTiff returned for the file. Throws std::invalid_argument when the window is empty or
// not inside geometry's samples, and std::runtime_error, with the file's name and libtiff's messages, when the file
// cannot be opened, no longer has geometry's size, or cannot be read.
[[nodiscard]] std::vector<float> readWindow(const std::filesystem::path& path, const GridGeometry& geometry,
                                            const PixelWindow& window);

}  // namespace demdl::mesh
