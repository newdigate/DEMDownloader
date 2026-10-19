#pragma once

#include <cstdint>
#include <filesystem>

#include "mesh/SolidMesh.h"

namespace demdl::mesh {

// Writes the terrain's top surface (what surfaceMesh makes, not the whole solid) as a binary little-endian PLY (spec
// 5.5 and 7.3), the hand-off to Blender and usable in any 3D tool. The header is
//   ply
//   format binary_little_endian 1.0
//   comment dem2mesh <version>
//   element vertex N
//   property float x, y, z, s, t (one line each)
//   element face M
//   property list uchar uint vertex_indices
//   end_header
// each line ending in "\n", followed by N vertices of five float32 (x, y, z, s, t) and M faces of a uchar 3 and three
// uint32, the vertices and triangles of surface in their order. s and t are a planar texture map over the window,
//   s = x / widthUnits,  t = y / heightUnits,
// which Blender's importer reads as the object's UV map, so that draping an aerial photo over the relief later needs no
// change to the mesh. widthUnits and heightUnits are the window's size in the mesh's own units (the window in metres
// times the scale), so that s and t span 0 to 1 over the window; they are passed in rather than taken from the vertices
// because the vertices' extent is that window only to within float rounding.
//
// Written to <path>.tmp and renamed over path once complete, as writeBinaryStl does, with the same failures: a
// std::runtime_error naming the file and the operating system's reason when it cannot be written. Throws
// std::invalid_argument, creating no file, when widthUnits or heightUnits is not above 0 and finite, or a triangle
// names a vertex the surface does not have.
void writeBinaryPly(const Mesh& surface, const std::filesystem::path& path, double widthUnits, double heightUnits);

// The size in bytes of the file writeBinaryPly writes for a surface of `vertices` vertices and `faces` triangles: the
// header above for those counts, then 20 bytes a vertex and 13 a face. For the dry run and the summary, which give the
// size before the file is written or without writing it.
[[nodiscard]] std::uint64_t plyFileBytes(std::uint64_t vertices, std::uint64_t faces);

}  // namespace demdl::mesh
