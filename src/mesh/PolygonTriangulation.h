#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "mesh/Predicates.h"

namespace demdl::mesh {

// The triangles of a simple polygon given as its vertices in counter-clockwise order (orient2d positive turns, with
// x to the right and y up), as triples of indices into `loop`, each counter-clockwise, n - 2 of them. Vertices may be
// collinear with their neighbours (a straight run of a rectilinear outline keeps samples along it); such a vertex is
// never an ear itself, and no triangle has another vertex of the polygon inside it or on one of its edges, so the
// result has no T-junction and no triangle without area. Ear clipping with the exact predicates, deterministic:
// the first ear in index order is clipped at every step. Coordinates must lie within kMostPredicateCoordinate.
// Throws std::invalid_argument when loop has fewer than 3 vertices, repeats a vertex, is clockwise or has no area,
// or no ear can be found (a self-intersecting polygon).
//
// The rule, so that the output can be told in advance: of the vertices still in the polygon, the one of the lowest
// index that is an ear is cut off, and so on until two are left. A vertex is an ear when it turns counter-clockwise
// strictly (orient2d of its predecessor, itself and its successor is positive) and no other vertex still in the polygon
// lies in the closed triangle of the three, inside it or on any of its edges. The triple written for it is (its
// predecessor, itself, its successor) in the polygon as it then stands, and the triples come in the order the vertices
// were cut off. What is cut is the lowest-index ear of the polygon as it then stands, not the first ear found from
// where the last cut was made: a search that resumed at the cut vertex's predecessor would give other triangles, valid
// too but not these.
//
// What follows from the rule:
// - No triangle has a vertex of the polygon inside it or on an edge. When it was cut, the polygon that was left had
//   none in its closed triangle, and every vertex cut earlier lies on the far side of the diagonal that closed its own
//   ear, outside what is left. So the triangles tile the polygon, and two that share an edge share it corner to corner.
// - A simple polygon of positive area always has an ear while it has three or more vertices (the two-ears theorem,
//   which holds with vertices in line as well: they are never ears themselves, the others supply them), and what is
//   left after a cut is again such a polygon. So no simple polygon is refused for its shape, slivers, long straight
//   runs and spirals included.
// - Every test is orient2d on whole numbers, exact within the bound, so the same loop gives the same triangles on every
//   platform; nothing is rounded.
//
// What it does not do: look for a polygon that crosses itself. One that does is refused when its area is not positive
// or when no ear is left to cut, and otherwise is cut into n - 2 triangles that overlap. The ears are kept by rules
// that hold for simple polygons, so for such a polygon the set can miss an ear that a search would find. The caller
// passes a simple polygon. A debug build asserts the coordinate bound, as the predicates do.
//
// The cost: telling whether a vertex is an ear takes a look at every vertex that remains, so the ears are found once
// and kept. They are all tested at the start, the lowest is cut at each step, and only the two neighbours of the vertex
// cut are tested again: an ear stays one until a neighbour of it is cut, and cutting an ear makes an ear of no vertex
// but its two neighbours (for a simple polygon; the reasoning is in the source). So a loop of n vertices takes at most
// 3 n ear tests, each a look at no more than n vertices: under 2 n^2 looks in all, and about n^2 on the outlines
// tried, wherever the loop starts. Memory is 8 bytes a vertex for the list of the vertices left, 4 for the check for a
// repeated vertex, a node of the set (a few dozen bytes) for each ear, and 12 a triangle for the result.
[[nodiscard]] std::vector<std::array<std::uint32_t, 3>> triangulatePolygon(std::span<const GridPoint> loop);

}  // namespace demdl::mesh
