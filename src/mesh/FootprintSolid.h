#pragma once

#include <cstdint>
#include <vector>

#include "mesh/Simplifier.h"
#include "mesh/SolidMesh.h"

namespace demdl::mesh {

// The printable solid of a block with footprint pieces (spec 5.3, 6): one closed shell for each piece, made of the
// piece's part of the surface, two wall triangles under every edge of its outline, and a base at z = planeZ. It is for
// a surface that ends in outlines of any shape what solidFromSurface is for one that ends in a rectangle's four sides.
//
// `surface` is the block's surface in model units, a sheet of triangles counter-clockwise seen from above. `pieces`
// are the outlines BlockSimplifier gave for it (Simplifier.h), their vertex numbers numbering the vertices of
// `surface`: each outline lists its vertices counter-clockwise seen from above, closed without repeating the first,
// with the piece on the left of every edge, and `points` has the same vertices' (column, row) in the block, rows
// counted southward.
//
// The solid starts with the surface's vertices and triangles, unchanged and in the same order, so the surface is a
// prefix of the solid. Then, in the order of `pieces`, each piece adds
// - to the vertices, a bottom copy of each vertex of its outline, in the outline's order: the vertex's x and y at
//   z = planeZ. With S the surface's vertex count and n0, n1, ... the numbers of vertices of the outlines, the k-th
//   vertex of piece p has its copy at vertex number S + n0 + ... + n(p-1) + k;
// - to the triangles, first the walls, then the base. For each edge of the outline in the outline's order, from its
//   k-th vertex a to the next b (the first one, after the last), with a' and b' the copies of a and b, the wall has
//   the triangles (a, a', b') and (a, b', b), 2 n of them for an outline of n vertices; then the base's n - 2.
//
// A wall faces out. The outline runs counter-clockwise with the piece on its left, so the outside is on the right of
// every edge a -> b. Seen from there, standing before the wall, a is on the left and b on the right, a' under a and b'
// under b, and both (a, a', b') and (a, b', b) run counter-clockwise: they face the viewer, out of the piece. They cut
// the flat quadrilateral a, a', b', b along its diagonal from a to b'. Each has an area, half the edge's length times
// the height of a (or of b) over the plane. The order the corners are named in matters: written the other way round, a
// wall faces into the solid. The rule is solidFromSurface's, whose walls are cut by SolidMesh.cpp's normalOf: a
// triangle faces the side from which its vertices run counter-clockwise.
//
// The base is the outline's polygon at z = planeZ. triangulatePolygon (PolygonTriangulation.h) cuts it from `points`
// with their rows negated, which turns the outline counter-clockwise for its predicates, into n - 2 triangles over the
// n copies and no other vertex; each of its triples is then written backwards, clockwise seen from above, so that it
// faces down. With no vertex of its own, the base shares each bottom edge of the walls, with no T-junction anywhere,
// and does so for a vertex in line with its neighbours as well: a run of the outline keeps the samples its profile
// needs (spec 5.2), so such vertices are common, and triangulatePolygon uses one as a corner of a triangle but never
// cuts it off as an ear, and leaves no triangle without area.
//
// Each shell is closed, every edge shared by two triangles that run along it in opposite directions: an outline edge
// a -> b by the surface triangle of the piece beside it, which runs a -> b, and the wall's (a, b', b), which runs
// b -> a; a bottom edge a' -> b' by the wall's (a, a', b') and the base, which runs b' -> a'; the upright edge from a
// to a' by the wall of the edge that starts at a, and the one from a' to a by the wall of the edge that ends there.
// The shell encloses the prisms under its part of the surface: the sum over its surface triangles of their mean height
// over the plane times their area seen from above.
//
// No two pieces of a block touch: footprintCells joins two pieces that would meet at a vertex, by putting in one of
// the two empty cells there (Footprint.h), so that every block's shells stay apart. Outlines that did touch, holding
// the same vertex of the surface, would make two shells that meet at that one top vertex, each with a copy of its own
// under it, and in this mesh every edge would still have its two triangles: checkSolid would find no bad edge and the
// volumes would add up. The geometry is another matter. Both shells would have walls standing on the upright line
// under the contact, so the two prisms would touch along a line, not at a point; an STL has no shared vertices, and a
// reader that merges the corners at one position (readBinaryStl, and the importer that blender_scene.py's check_stl
// uses) joins the two copies under the contact and finds that line to be one edge of four faces, a non-manifold edge
// for each contact. No numbering or cutting of the walls changes that, which is why the footprint joins such pieces.
//
// The surface is any sheet of triangles whose only free edges are the pieces' outlines, each one's edge a -> b borne by
// a triangle that runs a -> b, as BlockSimplifier makes it. Its triangles are not looked at, as solidFromSurface does
// not look at them: a sheet with another hole, or an outline edge without its triangle, makes a solid that is not
// closed, which checkSolid shows. Vertices that no outline names, the inside's, may be any in number, and vertices that
// no triangle uses stay where they are. The outline's `points` and its vertices' positions are not held to each other
// here: the walls follow the positions, the base follows the points, and the caller numbers both alike.
//
// Throws std::invalid_argument, when the solid cannot be made, with a message that says what is wrong: the piece by its
// place in `pieces`, from 0, and the vertex or point at fault where there is one. The checks are made in this order:
// 1. when the surface and the copies would have more vertices together than 32-bit vertex numbers can count;
// 2. piece by piece, when an outline has fewer than 3 vertices, a different number of points from its vertices, a
//    vertex number that is not one of the surface's, or a point beyond kMostPredicateCoordinate in either coordinate,
//    the bound the exact predicates need;
// 3. when planeZ is not finite;
// 4. when a vertex of the surface, on an outline or not, is not above planeZ (a NaN one included), a wall having to
//    reach down from every vertex of an outline to the base;
// 5. piece by piece, as each is built, when triangulatePolygon refuses its points: an outline that is not
//    counter-clockwise seen from above (clockwise, or without area), that repeats a point, or that leaves no ear to
//    cut. An outline that crosses itself is not looked for: it is refused only when one of those holds, and otherwise
//    cut into overlapping triangles (PolygonTriangulation.h), so the caller passes a simple one, as BlockSimplifier's
//    outlines are.
// Nothing is returned when it throws, whichever check it was.
//
// Time and memory: linear in the surface and the outlines, plus triangulatePolygon, quadratic in an outline's vertices
// at worst (PolygonTriangulation.h). The solid is a copy of the surface with the shells added, so the two are held
// together until the caller drops one.
[[nodiscard]] Mesh solidFromPieces(const Mesh& surface, const std::vector<PieceOutline>& pieces, float planeZ);

// surfaceTriangles + 3 n - 2 for each piece of n vertices: 2 n wall triangles and the base's n - 2. It only adds up, in
// 64 bits, from the outlines' sizes. A piece of fewer than 3 vertices, which solidFromPieces refuses, makes no shell
// and adds nothing.
[[nodiscard]] std::uint64_t solidTriangleCount(const std::vector<PieceOutline>& pieces,
                                               std::uint64_t surfaceTriangles) noexcept;

}  // namespace demdl::mesh
