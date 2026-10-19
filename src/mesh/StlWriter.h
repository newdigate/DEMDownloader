#pragma once

#include <filesystem>
#include <string_view>

#include "mesh/SolidMesh.h"

namespace demdl::mesh {

// Writes mesh as a binary STL (spec 5.5), the file a slicer reads:
// - 80 bytes of header: header cut to 80 bytes or padded with zeros. It should not begin with "solid": some readers
//   take a file that does for an ASCII STL, whatever follows.
// - a uint32 count of triangles;
// - per triangle, 50 bytes: the unit normal as three float32, the three vertices in the triangle's order as nine
//   float32, and a uint16 attribute byte count of 0. The normal is (b - a) x (c - a) scaled to length 1, worked out in
//   double, so for a triangle wound counter-clockwise seen from outside, as Task 7's solid is, it points out of the
//   solid. A triangle with no area, or with a coordinate that is not a number, has no direction to give and gets (0, 0,
//   0), which readers usually take to mean "work it out from the vertices".
// Every number is little-endian whatever the host. The format has no shared vertices, so each triangle repeats its
// corners; readBinaryStl merges them again.
//
// The file is written to <path>.tmp and renamed over path once complete, as DemRaster::save does, so path is never half
// a file and keeps what it held if the write fails; the .tmp does not outlive a failure.
//
// Throws std::runtime_error naming the file and the operating system's reason when it cannot be written: a missing
// directory, a full disk, a destination that cannot be replaced. Throws std::invalid_argument, creating no file, when a
// triangle names a vertex the mesh does not have or the mesh has more triangles than a uint32 counts.
void writeBinaryStl(const Mesh& mesh, const std::filesystem::path& path, std::string_view header);

// Reads a binary STL as writeBinaryStl writes it, for the tests and the end-to-end check (not a general importer). The
// vertices of equal position are merged into one, numbered in order of first appearance, so that checkSolid can run on
// the result; minus zero is the same position as zero. The stored normals, the header and the attribute bytes are
// ignored.
//
// Throws std::runtime_error naming the file when it cannot be read, or when its size is not 84 bytes plus 50 for each
// triangle its count declares: an ASCII STL, a cut-off file and one with bytes to spare all fail that.
[[nodiscard]] Mesh readBinaryStl(const std::filesystem::path& path);

}  // namespace demdl::mesh
