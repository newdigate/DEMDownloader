#include "mesh/Simplifier.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mesh/Predicates.h"
#include "mesh/ProfileSimplifier.h"

namespace demdl::mesh {

static_assert(kMostChunkCells <= kMostPredicateCoordinate,
              "a chunk's own coordinates run from 0 to its side, which the predicates must be exact for");

namespace {

// Vertex numbers are 32 bits, and the solid adds its four bottom corners to the surface's vertices.
constexpr std::uint64_t kMostVertices = std::uint64_t{std::numeric_limits<std::uint32_t>::max()} + 1 - 4;

std::string formatNumber(double value, int digits) {
  char text[40];
  std::snprintf(text, sizeof text, "%.*g", digits, value);
  return text;
}

std::string gridSize(int cols, int rows) { return std::to_string(cols) + " x " + std::to_string(rows); }

// The grid as simplifyGrid is given it: cols x rows heights, row by row from the north-west corner.
struct GridView {
  const float* heights;
  int cols, rows;

  // Where row `row` starts: the height of its first column, the rest of the row following it.
  const float* rowStart(int row) const {
    return heights + static_cast<std::size_t>(row) * static_cast<std::size_t>(cols);
  }
  float at(int column, int row) const { return rowStart(row)[column]; }
};

// Where the chunks' vertices go: the surface in model units. Each vertex is placed by surfaceMesh's own expressions,
// operation for operation, so that it lands on the very floats the full grid's vertex at that sample would.
class SurfaceBuilder {
 public:
  SurfaceBuilder(GridView grid, double widthM, double heightM, double zScale, double scale, double levelM)
      : grid_(grid), widthM_(widthM), heightM_(heightM), zScale_(zScale), scale_(scale), levelM_(levelM) {}

  // Appends the vertex of sample (column, row) and returns its number.
  std::uint32_t add(int column, int row) {
    const double x = column * widthM_ / (grid_.cols - 1) * scale_;
    const double y = (grid_.rows - 1 - row) * heightM_ / (grid_.rows - 1) * scale_;
    const double z = std::max(static_cast<double>(grid_.at(column, row)), levelM_) * zScale_ * scale_;
    mesh.vertices.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    return static_cast<std::uint32_t>(mesh.vertices.size() - 1);
  }

  // The number of the vertex of sample (column, row) on a chunk's outline. Each chunk that has the sample asks for it,
  // the two beside a side or the up to four around a corner: the first to ask appends the vertex and the others are
  // given its number, so that it is in the output once. Only outline samples are held, so the map is no larger than
  // the samples the chunks' sides keep.
  std::uint32_t addShared(int column, int row) {
    const std::uint64_t sample = static_cast<std::uint64_t>(row) * static_cast<std::uint64_t>(grid_.cols) +
                                 static_cast<std::uint64_t>(column);
    const auto [found, isNew] = shared_.try_emplace(sample);
    if (isNew) found->second = add(column, row);
    return found->second;
  }

  Mesh mesh;

 private:
  GridView grid_;
  double widthM_, heightM_, zScale_, scale_, levelM_;
  std::unordered_map<std::uint64_t, std::uint32_t> shared_;  // by row cols + column, the outline samples' vertices
};

// The rectangle of samples a chunk covers, its sides included: columns firstColumn to lastColumn and rows firstRow to
// lastRow of the grid.
struct ChunkRect {
  int firstColumn, firstRow, lastColumn, lastRow;
};

// The samples a chunk's sides keep, each side as its line was simplified: simplifyProfile settles a tie by the order
// the heights come in, so a row is always simplified from west to east and a column from north to south, whichever way
// a chunk's outline walks it, and two chunks that share a side keep the same samples on it. The north and south sides
// are grid columns, the west and east sides grid rows, each from corner to corner.
struct ChunkSides {
  std::span<const int> north, south, west, east;
};

// The columns kept along row `row` from firstColumn to lastColumn, the row simplified from west to east.
std::vector<int> keptAlongRow(GridView grid, int row, int firstColumn, int lastColumn, double toleranceM) {
  const std::span<const float> line(grid.rowStart(row) + firstColumn,
                                    static_cast<std::size_t>(lastColumn - firstColumn + 1));
  std::vector<int> kept = simplifyProfile(line, toleranceM);
  for (int& k : kept) k += firstColumn;
  return kept;
}

// The rows kept along column `column` from firstRow to lastRow, the column simplified from north to south. A column's
// heights lie a row apart in the grid, so they are copied into a line of their own first.
std::vector<int> keptAlongColumn(GridView grid, int column, int firstRow, int lastRow, double toleranceM) {
  std::vector<float> line;
  line.reserve(static_cast<std::size_t>(lastRow - firstRow + 1));
  for (int row = firstRow; row <= lastRow; ++row) line.push_back(grid.at(column, row));
  std::vector<int> kept = simplifyProfile(line, toleranceM);
  for (int& k : kept) k += firstRow;
  return kept;
}

// The samples each chunk's side keeps, simplified once for the one or two chunks that have the side. A side two chunks
// share is the same segment of a grid line for both, known by its axis, its line and its range, so both are handed the
// same samples, which become the same vertices along their common edge. Only the sides' samples are held, and what the
// cache hands out stays where it is while the cache lives.
class SideCache {
 public:
  SideCache(GridView grid, double toleranceM) : grid_(grid), toleranceM_(toleranceM) {}

  // The columns kept along row `row` from firstColumn to lastColumn, the row simplified from west to east.
  std::span<const int> alongRow(int row, int firstColumn, int lastColumn) {
    const auto [side, isNew] = kept_.try_emplace(Side{true, row, firstColumn, lastColumn});
    if (isNew) side->second = keptAlongRow(grid_, row, firstColumn, lastColumn, toleranceM_);
    return side->second;
  }

  // The rows kept along column `column` from firstRow to lastRow, the column simplified from north to south.
  std::span<const int> alongColumn(int column, int firstRow, int lastRow) {
    const auto [side, isNew] = kept_.try_emplace(Side{false, column, firstRow, lastRow});
    if (isNew) side->second = keptAlongColumn(grid_, column, firstRow, lastRow, toleranceM_);
    return side->second;
  }

 private:
  // A side: along a row or a column, which one, and from where to where along it.
  struct Side {
    bool alongRow;
    int line, first, last;
    auto operator<=>(const Side&) const = default;
  };

  GridView grid_;
  double toleranceM_;
  std::map<Side, std::vector<int>> kept_;  // a map of nodes, so that a side's samples stay where they are
};

// floor(n / d) for d > 0, which C++'s division, rounding towards zero, is not when n is negative.
std::int64_t floorDiv(std::int64_t n, std::int64_t d) {
  const std::int64_t q = n / d;
  return n % d != 0 && n < 0 ? q - 1 : q;
}

using FaceId = std::int32_t;
constexpr FaceId kOutside = -1;  // what lies across an edge of the chunk's outline

// The greedy triangulation of one chunk (spec 4.4), in the chunk's own coordinates: a vertex is a GridPoint whose x is
// its column and y its row, both counted from the chunk's north-west corner, so y grows to the south. orient2d takes y
// to grow to the north, so a face whose corners turn counter-clockwise for orient2d runs clockwise on the map; every
// face is kept that way round, and is turned back to counter-clockwise seen from above when it is written out.
//
// The outline comes first: the four corners make two faces, cut along the diagonal from the north-west corner to the
// south-east one as a grid's cells are, and each sample a side keeps is inserted on the outline edge it falls on, which
// splits the face there in two. Then the interior: every face knows its worst sample, the one furthest from its plane
// vertically, and a heap of the faces gives the worst of all; it is inserted, and so on until none is further than the
// tolerance. A sample strictly inside a face splits it in three; one on an edge between two faces splits both, in four.
// After each insertion Lawson's flips restore the empty-circle property around the new vertex: an edge is flipped when
// the vertex across it lies strictly inside the circle of the face holding the new vertex (inCircle > 0, never at 0,
// so that four points on one circle are left as they are), and never an edge of the outline. Only the faces an
// insertion makes or changes are scanned again.
//
// The samples on the chunk's sides are never scanned: simplifyProfile has put each of them within the tolerance of the
// outline's edges, which are never split again, so none is ever inserted from the interior, and the sides keep exactly
// the samples they were given. That is what lets two chunks share a side. A face's own corners are not scanned either:
// they lie on the surface already, and so cannot be inserted twice, however the rounding falls.
class ChunkTin {
 public:
  ChunkTin(GridView grid, ChunkRect rect, double toleranceM, SurfaceBuilder& out)
      : grid_(grid),
        rect_(rect),
        width_(rect.lastColumn - rect.firstColumn),
        height_(rect.lastRow - rect.firstRow),
        toleranceM_(toleranceM),
        out_(out) {}

  // Triangulates the chunk, appends its vertices and triangles to the output and returns its outline, as vertex numbers
  // of the output. The vertices come in the order they are inserted: the corners (south-west, south-east, north-east,
  // north-west, the first vertices of the outline's sides), then the samples the sides keep, side by side in the
  // outline's order and along each side as the outline walks it, then the interior. An outline vertex that a chunk
  // before this one has put in the output already is not put there again.
  Boundary build(const ChunkSides& sides) {
    const std::array<std::uint32_t, 4> corner = {addVertex({0, height_}), addVertex({width_, height_}),
                                                 addVertex({width_, 0}), addVertex({0, 0})};
    const auto [southWest, southEast, northEast, northWest] = corner;
    const FaceId upper = newFace();
    const FaceId lower = newFace();
    write(upper, {northWest, northEast, southEast}, {kOutside, kOutside, lower});
    write(lower, {northWest, southEast, southWest}, {upper, kOutside, kOutside});

    // The outline's edges along a side run from its last corner back to its first, the faces turning
    // counter-clockwise for orient2d: so the next sample along the side lies on the outline edge that starts at the
    // side's last corner. No face is scanned until the outline is done, when every face is, so the faces each
    // insertion writes are forgotten as soon as it is legalised: a side inserted after the one opposite it flips each
    // new sample's fan across to that side, about a quarter of the product of the two sides' samples in all, and
    // noting every face written would hold two numbers a flip.
    const std::array<std::vector<GridPoint>, 4> walks = outlineWalks(sides);
    Boundary outline;
    for (std::size_t s = 0; s < 4; ++s) {
      const std::uint32_t end = corner[(s + 1) % 4];
      const std::vector<GridPoint>& walk = walks[s];
      assert(walk.size() >= 2 && same(walk.front(), at(corner[s])) && same(walk.back(), at(end)) &&
             "a side must run from its corner to the next");
      outline.sides[s].push_back(vertices_[corner[s]].number);
      for (std::size_t k = 1; k + 1 < walk.size(); ++k) {
        const std::uint32_t p = addVertex(walk[k]);
        const FaceId f = outlineFrom_[end];
        splitEdge(f, indexOf(faces_[static_cast<std::size_t>(f)], end), p);
        legalise();
        touched_.clear();
        outline.sides[s].push_back(vertices_[p].number);
      }
      outline.sides[s].push_back(vertices_[end].number);
    }
    touched_.clear();  // the four corners' first two faces

    for (FaceId f = 0; f < static_cast<FaceId>(faces_.size()); ++f) rescan(f);
    while (!heap_.empty()) {
      std::pop_heap(heap_.begin(), heap_.end(), comesAfter);
      const Candidate top = heap_.back();
      heap_.pop_back();
      if (top.version == faces_[static_cast<std::size_t>(top.face)].version) insert(top.face);
    }

    for (const Face& face : faces_) {
      out_.mesh.triangles.push_back(
          {vertices_[face.v[0]].number, vertices_[face.v[2]].number, vertices_[face.v[1]].number});
    }
    return outline;
  }

 private:
  struct Vertex {
    GridPoint at;          // in the chunk's coordinates
    double height;         // metres
    std::uint32_t number;  // in the output
  };

  struct Face {
    std::array<std::uint32_t, 3> v{};  // its corners, turning counter-clockwise for orient2d
    std::array<FaceId, 3> across{};    // across[k]: the face across the edge from v[k] to v[(k + 1) % 3], or kOutside
    GridPoint worst{0, 0};             // the sample furthest from the face's plane, when error is not negative
    double error = -1;                 // that sample's vertical distance from the plane, metres; -1 for no sample
    std::uint32_t version = 0;         // counts the face's rewrites, so that a heap entry of an earlier one is known
  };

  // What a face's worst sample was when the face was last scanned: an entry of the heap.
  struct Candidate {
    double error;
    std::uint32_t sample;  // its row times the chunk's width in samples, plus its column
    FaceId face;
    std::uint32_t version;
  };

  // The heap's order: whether p comes after q. The larger error first; of equal errors the sample in the smaller row,
  // then the smaller column, as a face's own scan picks its worst sample; then the smaller face and its newer version,
  // so that no two entries are equal and the order of insertions does not depend on how a heap breaks ties.
  static bool comesAfter(const Candidate& p, const Candidate& q) {
    if (p.error != q.error) return p.error < q.error;
    if (p.sample != q.sample) return p.sample > q.sample;
    if (p.face != q.face) return p.face > q.face;
    return p.version < q.version;
  }

  static bool same(GridPoint p, GridPoint q) { return p.x == q.x && p.y == q.y; }

  GridPoint at(std::uint32_t v) const { return vertices_[v].at; }

  // The samples the sides keep, in the chunk's coordinates, each side from its first corner to its last as the
  // outline walks it: the south side from west to east, the east side from south to north, the north side from east
  // to west and the west side from north to south. The east and north sides were simplified the other way, so their
  // samples are taken in reverse.
  std::array<std::vector<GridPoint>, 4> outlineWalks(const ChunkSides& sides) const {
    std::array<std::vector<GridPoint>, 4> walks;
    for (const int column : sides.south) walks[0].push_back({column - rect_.firstColumn, height_});
    for (auto row = sides.east.rbegin(); row != sides.east.rend(); ++row) {
      walks[1].push_back({width_, *row - rect_.firstRow});
    }
    for (auto column = sides.north.rbegin(); column != sides.north.rend(); ++column) {
      walks[2].push_back({*column - rect_.firstColumn, 0});
    }
    for (const int row : sides.west) walks[3].push_back({0, row - rect_.firstRow});
    return walks;
  }

  // Adds the vertex of sample p. A sample on the chunk's outline is on a side the chunk beside it has too, with the
  // same samples kept, or on a corner up to four chunks share, so its vertex goes into the output once, with whichever
  // chunk reaches it first. A sample inside is the chunk's own.
  std::uint32_t addVertex(GridPoint p) {
    const int column = rect_.firstColumn + p.x;
    const int row = rect_.firstRow + p.y;
    const bool onOutline = p.x == 0 || p.x == width_ || p.y == 0 || p.y == height_;
    const std::uint32_t number = onOutline ? out_.addShared(column, row) : out_.add(column, row);
    vertices_.push_back({p, static_cast<double>(grid_.at(column, row)), number});
    return static_cast<std::uint32_t>(vertices_.size() - 1);
  }

  FaceId newFace() {
    faces_.emplace_back();
    return static_cast<FaceId>(faces_.size() - 1);
  }

  static int indexOf(const Face& face, std::uint32_t v) {
    for (int k = 0; k < 3; ++k) {
      if (face.v[static_cast<std::size_t>(k)] == v) return k;
    }
    assert(false && "the vertex must be a corner of the face");
    return 0;
  }

  // Gives face f new corners and neighbours. Its version moves on, which retires its entries in the heap, and it is
  // noted for a rescan. Where an edge of the outline starts at a corner of the chunk, the face is noted as the one
  // that holds it.
  void write(FaceId f, std::array<std::uint32_t, 3> v, std::array<FaceId, 3> across) {
    assert(orient2d(at(v[0]), at(v[1]), at(v[2])) > 0 && "a face must turn counter-clockwise for orient2d");
    Face& face = faces_[static_cast<std::size_t>(f)];
    face.v = v;
    face.across = across;
    ++face.version;
    touched_.push_back(f);
    for (std::size_t k = 0; k < 3; ++k) {
      if (across[k] == kOutside && v[k] < 4) outlineFrom_[v[k]] = f;
    }
  }

  // Tells face f, which had `from` across one of its edges, that `to` is there now. Its shape does not change, so it
  // needs no rescan.
  void repoint(FaceId f, FaceId from, FaceId to) {
    if (f == kOutside) return;
    for (FaceId& next : faces_[static_cast<std::size_t>(f)].across) {
      if (next == from) {
        next = to;
        return;
      }
    }
    assert(false && "a neighbour must point back");
  }

  // Splits face f = (a, b, c) at the new vertex p strictly inside it, into (a, b, p), (b, c, p) and (c, a, p).
  void splitInside(FaceId f, std::uint32_t p) {
    const Face face = faces_[static_cast<std::size_t>(f)];
    const auto [a, b, c] = face.v;
    const auto [acrossAB, acrossBC, acrossCA] = face.across;
    const FaceId g = newFace();
    const FaceId h = newFace();
    write(f, {a, b, p}, {acrossAB, g, h});
    write(g, {b, c, p}, {acrossBC, h, f});
    write(h, {c, a, p}, {acrossCA, f, g});
    repoint(acrossBC, f, g);
    repoint(acrossCA, f, h);
    pending_.insert(pending_.end(), {f, g, h});
  }

  // Splits face f at the new vertex p on its edge k, from a = v[k] to b = v[k + 1], into (b, c, p) and (c, a, p), c
  // being its third corner; and the face across that edge, (b, a, d), into (a, d, p) and (d, b, p). On the outline
  // there is no face across, and the edge becomes two edges of the outline.
  void splitEdge(FaceId f, int k, std::uint32_t p) {
    const Face face = faces_[static_cast<std::size_t>(f)];
    const auto corner = [&](int i) { return face.v[static_cast<std::size_t>((k + i) % 3)]; };
    const auto neighbour = [&](int i) { return face.across[static_cast<std::size_t>((k + i) % 3)]; };
    const std::uint32_t a = corner(0), b = corner(1), c = corner(2);
    const FaceId u = neighbour(0), acrossBC = neighbour(1), acrossCA = neighbour(2);
    const FaceId g = newFace();
    if (u == kOutside) {
      write(f, {b, c, p}, {acrossBC, g, kOutside});
      write(g, {c, a, p}, {acrossCA, kOutside, f});
      repoint(acrossCA, f, g);
      pending_.insert(pending_.end(), {f, g});
      return;
    }
    const Face other = faces_[static_cast<std::size_t>(u)];
    const int j = indexOf(other, b);
    assert(other.v[static_cast<std::size_t>((j + 1) % 3)] == a && "the face across must run the edge the other way");
    const std::uint32_t d = other.v[static_cast<std::size_t>((j + 2) % 3)];
    const FaceId acrossAD = other.across[static_cast<std::size_t>((j + 1) % 3)];
    const FaceId acrossDB = other.across[static_cast<std::size_t>((j + 2) % 3)];
    const FaceId h = newFace();
    write(f, {b, c, p}, {acrossBC, g, h});
    write(g, {c, a, p}, {acrossCA, u, f});
    write(u, {a, d, p}, {acrossAD, h, g});
    write(h, {d, b, p}, {acrossDB, f, u});
    repoint(acrossCA, f, g);
    repoint(acrossDB, u, h);
    pending_.insert(pending_.end(), {f, g, u, h});
  }

  // Lawson's flips around the newest vertex p. Every face waiting in pending_ has p as its third corner, so its first
  // edge, from v[0] to v[1], is the one opposite p, and only such edges can have lost the empty-circle property. When
  // the vertex q across that edge lies strictly inside the circle through the face's corners, the two faces' common
  // edge is flipped: (x, y, p) and (y, x, q) become (x, q, p) and (q, y, p), each with p as its third corner again,
  // and their edges opposite p wait in turn. q lying strictly inside makes the four points a strictly convex
  // quadrilateral, so both new faces have an area.
  void legalise() {
    while (!pending_.empty()) {
      const FaceId f = pending_.back();
      pending_.pop_back();
      const Face face = faces_[static_cast<std::size_t>(f)];
      const FaceId g = face.across[0];
      if (g == kOutside) continue;
      const auto [x, y, p] = face.v;
      const Face other = faces_[static_cast<std::size_t>(g)];
      const int j = indexOf(other, y);
      assert(other.v[static_cast<std::size_t>((j + 1) % 3)] == x && "the face across must run the edge the other way");
      const std::uint32_t q = other.v[static_cast<std::size_t>((j + 2) % 3)];
      if (inCircle(at(x), at(y), at(p), at(q)) <= 0) continue;
      const FaceId acrossXQ = other.across[static_cast<std::size_t>((j + 1) % 3)];
      const FaceId acrossQY = other.across[static_cast<std::size_t>((j + 2) % 3)];
      const FaceId acrossYP = face.across[1];
      const FaceId acrossPX = face.across[2];
      write(f, {x, q, p}, {acrossXQ, g, acrossPX});
      write(g, {q, y, p}, {acrossQY, acrossYP, f});
      repoint(acrossXQ, g, f);
      repoint(acrossYP, f, g);
      pending_.push_back(f);
      pending_.push_back(g);
    }
  }

  // Narrows [lo, hi] on row y, a row of the face's bounding box, to the columns x where (x, y) lies on the inner side
  // of the face's edge from p to q or on it: orient2d(p, q, (x, y)) >= 0. There orient2d is (q.x - p.x)(y - p.y) -
  // (q.y - p.y)(x - p.x), linear in x, so the bound is one division of whole numbers, rounded inwards, and exact. An
  // edge along a row bounds no column: the face lies to one side of it, the side every row of its bounding box is on.
  static void clip(GridPoint p, GridPoint q, int y, std::int64_t& lo, std::int64_t& hi) {
    const std::int64_t rise = std::int64_t{q.y} - p.y;
    const std::int64_t reach = (std::int64_t{q.x} - p.x) * (std::int64_t{y} - p.y);
    assert((rise != 0 || reach >= 0) && "a row of the bounding box is on the inner side of an edge along a row");
    if (rise > 0) {
      hi = std::min(hi, p.x + floorDiv(reach, rise));
    } else if (rise < 0) {
      lo = std::max(lo, p.x - floorDiv(reach, -rise));
    }
  }

  // Finds face's worst sample: of the samples inside it or on its edges, its corners and the chunk's sides left out,
  // the one furthest from its plane, vertically; of equal distances the one in the smallest row, then the smallest
  // column, as the scan goes row by row from the north and along each row from the west and gives way only to a
  // strictly larger distance. The rows are those the face's bounding box covers and each row's span comes from the
  // three edges, so no sample outside the face is looked at.
  //
  // The plane is the barycentric mean of the corners' heights, with weights orient2d gives exactly in whole numbers.
  // Written from corner a it is a.height + perColumn (x - a.x) + perRow (y - a.y), perColumn and perRow being the
  // weights' slopes times the height differences, over twice the face's area. So a flat face gives its own height
  // exactly, and a sample's distance does not depend on the order the samples are scanned in.
  void scan(Face& face) const {
    const Vertex& a = vertices_[face.v[0]];
    const Vertex& b = vertices_[face.v[1]];
    const Vertex& c = vertices_[face.v[2]];
    const auto twiceArea = static_cast<double>(orient2d(a.at, b.at, c.at));
    const double riseB = b.height - a.height;
    const double riseC = c.height - a.height;
    const double perColumn =
        (static_cast<double>(c.at.y - a.at.y) * riseB + static_cast<double>(a.at.y - b.at.y) * riseC) / twiceArea;
    const double perRow =
        (static_cast<double>(a.at.x - c.at.x) * riseB + static_cast<double>(b.at.x - a.at.x) * riseC) / twiceArea;

    face.error = -1;
    const int top = std::max(1, std::min({a.at.y, b.at.y, c.at.y}));
    const int bottom = std::min(height_ - 1, std::max({a.at.y, b.at.y, c.at.y}));
    const std::int64_t west = std::max(1, std::min({a.at.x, b.at.x, c.at.x}));
    const std::int64_t east = std::min(width_ - 1, std::max({a.at.x, b.at.x, c.at.x}));
    for (int y = top; y <= bottom; ++y) {
      std::int64_t lo = west;
      std::int64_t hi = east;
      clip(a.at, b.at, y, lo, hi);
      clip(b.at, c.at, y, lo, hi);
      clip(c.at, a.at, y, lo, hi);
      // A corner of the face is an end of its row's span, the face being convex.
      for (const Vertex* v : {&a, &b, &c}) {
        if (v->at.y != y) continue;
        if (v->at.x == lo) {
          ++lo;
        } else if (v->at.x == hi) {
          --hi;
        }
      }
      if (lo > hi) continue;
      const float* heights = grid_.rowStart(rect_.firstRow + y) + rect_.firstColumn;
      const double base = a.height + perRow * (y - a.at.y);
      for (auto x = static_cast<int>(lo); x <= hi; ++x) {
        const double off = std::fabs(static_cast<double>(heights[x]) - (base + perColumn * (x - a.at.x)));
        if (off > face.error) {
          face.error = off;
          face.worst = {x, y};
        }
      }
    }
  }

  // Scans face f and, when its worst sample is beyond the tolerance, offers it to the heap. A face within the tolerance
  // is never offered, so the heap runs dry exactly when every face is within it.
  void rescan(FaceId f) {
    Face& face = faces_[static_cast<std::size_t>(f)];
    scan(face);
    if (face.error <= toleranceM_) return;
    const auto sample = static_cast<std::uint32_t>(face.worst.y) * static_cast<std::uint32_t>(width_ + 1) +
                        static_cast<std::uint32_t>(face.worst.x);
    heap_.push_back({face.error, sample, f, face.version});
    std::push_heap(heap_.begin(), heap_.end(), comesAfter);
  }

  // Inserts face f's worst sample, which lies inside f or on one of its edges and is none of its corners. The edge it
  // lies on, if any, is not on the outline, as the sample is not on the chunk's sides, so there is a face across it.
  void insert(FaceId f) {
    const Face face = faces_[static_cast<std::size_t>(f)];
    const GridPoint s = face.worst;
    const std::array<std::int64_t, 3> side = {orient2d(at(face.v[0]), at(face.v[1]), s),
                                              orient2d(at(face.v[1]), at(face.v[2]), s),
                                              orient2d(at(face.v[2]), at(face.v[0]), s)};
    assert(side[0] >= 0 && side[1] >= 0 && side[2] >= 0 && "the worst sample must lie in its face");
    const std::uint32_t p = addVertex(s);
    const auto edge = std::find(side.begin(), side.end(), 0);
    if (edge == side.end()) {
      splitInside(f, p);
    } else {
      const auto k = static_cast<int>(edge - side.begin());
      assert(face.across[static_cast<std::size_t>(k)] != kOutside && "an interior sample cannot be on the outline");
      splitEdge(f, k, p);
    }
    legalise();

    // An entry of a face that has changed since stays in the heap until it comes to the top and is passed over. An
    // insertion offers only the faces it made or changed, a handful, so the heap never holds more than a few entries
    // per face the chunk ends with.
    std::sort(touched_.begin(), touched_.end());
    touched_.erase(std::unique(touched_.begin(), touched_.end()), touched_.end());
    for (const FaceId g : touched_) rescan(g);
    touched_.clear();
  }

  GridView grid_;
  ChunkRect rect_;
  int width_, height_;  // the chunk's size in cells
  double toleranceM_;
  SurfaceBuilder& out_;
  std::vector<Vertex> vertices_;
  std::vector<Face> faces_;
  std::vector<Candidate> heap_;
  std::vector<FaceId> pending_;  // faces whose edge opposite the newest vertex is still to be legalised
  std::vector<FaceId> touched_;  // faces written since the last rescan
  std::array<FaceId, 4> outlineFrom_ = {kOutside, kOutside, kOutside, kOutside};  // by corner, the face holding the
                                                                                  // outline edge that starts there
};

// Simplifies one chunk of the grid, the rectangle rect, whose sides keep the samples sides lists: appends its
// triangles, and those of its vertices out does not have yet, to out, and returns its outline as vertex numbers of
// out's mesh.
Boundary simplifyChunk(GridView grid, ChunkRect rect, const ChunkSides& sides, double toleranceM, SurfaceBuilder& out) {
  return ChunkTin(grid, rect, toleranceM, out).build(sides);
}

// Appends a piece of the window's outline, the side of one chunk on the window's edge, to the side of the window it
// lies on. A piece after the first starts at the corner where the side so far ends, which the two chunks share and
// which is listed once.
void appendPiece(std::vector<std::uint32_t>& side, const std::vector<std::uint32_t>& piece) {
  assert(!piece.empty() && (side.empty() || side.back() == piece.front()) && "neighbouring chunks share a corner");
  side.insert(side.end(), piece.begin() + (side.empty() ? 0 : 1), piece.end());
}

}  // namespace

SimplifiedSurface simplifyGrid(const std::vector<float>& heights, int cols, int rows, double toleranceM, double widthM,
                               double heightM, double zScale, double scale, double levelM, int chunkCells) {
  if (cols < 2 || rows < 2) {
    throw std::invalid_argument("the grid must be at least 2 x 2 vertices, not " + gridSize(cols, rows));
  }
  const std::uint64_t count = static_cast<std::uint64_t>(cols) * static_cast<std::uint64_t>(rows);
  // Every sample may be kept, and vertex numbers are 32 bits with room left for the solid's four bottom corners:
  // judged before heights is looked at, as surfaceMesh judges it, and with its message.
  if (count > kMostVertices) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) +
                                " vertices is too large: 32-bit vertex numbers leave room for " +
                                std::to_string(kMostVertices) + " besides the four bottom corners");
  }
  if (heights.size() != count) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) + " vertices needs " + std::to_string(count) +
                                " heights, not " + std::to_string(heights.size()));
  }
  if (!std::isfinite(toleranceM) || toleranceM < 0.0) {
    throw std::invalid_argument("the tolerance must be finite and not negative, not " + formatNumber(toleranceM, 15));
  }
  if (chunkCells < 1 || chunkCells > kMostChunkCells) {
    throw std::invalid_argument("a chunk must be 1 to " + std::to_string(kMostChunkCells) + " cells a side, not " +
                                std::to_string(chunkCells));
  }
  // surfaceMesh's checks of the frame, with its messages.
  const auto positive = [](double value) { return value > 0 && std::isfinite(value); };
  if (!positive(widthM) || !positive(heightM)) {
    throw std::invalid_argument("the window must be wider and taller than 0 m and finite, not " +
                                formatNumber(widthM, 15) + " x " + formatNumber(heightM, 15) + " m");
  }
  if (!positive(zScale) || !positive(scale)) {
    throw std::invalid_argument("the z-scale and the scale must be above 0 and finite, not " +
                                formatNumber(zScale, 15) + " and " + formatNumber(scale, 15));
  }
  if (!std::isfinite(levelM)) throw std::invalid_argument("the level must be finite, not " + formatNumber(levelM, 15));
  const GridView grid{heights.data(), cols, rows};
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < cols; ++column) {
      if (!std::isfinite(grid.at(column, row))) {
        throw std::invalid_argument("the height at column " + std::to_string(column) + ", row " + std::to_string(row) +
                                    " is not finite");
      }
    }
  }

  // The lattice (spec 4.5): chunks of chunkCells cells a side from the north-west corner, those in the last column and
  // the last row of chunks cut short, taken row by row from the north and along each row from the west. A chunk's
  // sides come from the cache and its outline's vertices from out's shared numbers, so the chunks either side of a
  // seam meet on the same vertices, each in the output once. Counted so that no int overflows, whatever the grid.
  SurfaceBuilder out(grid, widthM, heightM, zScale, scale, levelM);
  SideCache sides(grid, toleranceM);
  const int across = (cols - 2) / chunkCells + 1;
  const int down = (rows - 2) / chunkCells + 1;
  std::array<std::vector<std::vector<std::uint32_t>>, 4> pieces;  // the window's sides, by chunk as they come
  for (int b = 0; b < down; ++b) {
    const int firstRow = b * chunkCells;
    const int lastRow = firstRow + std::min(chunkCells, rows - 1 - firstRow);
    for (int a = 0; a < across; ++a) {
      const int firstColumn = a * chunkCells;
      const int lastColumn = firstColumn + std::min(chunkCells, cols - 1 - firstColumn);
      const ChunkSides chunkSides{sides.alongRow(firstRow, firstColumn, lastColumn),
                                  sides.alongRow(lastRow, firstColumn, lastColumn),
                                  sides.alongColumn(firstColumn, firstRow, lastRow),
                                  sides.alongColumn(lastColumn, firstRow, lastRow)};
      Boundary outline =
          simplifyChunk(grid, ChunkRect{firstColumn, firstRow, lastColumn, lastRow}, chunkSides, toleranceM, out);
      if (b == down - 1) pieces[0].push_back(std::move(outline.sides[0]));
      if (a == across - 1) pieces[1].push_back(std::move(outline.sides[1]));
      if (b == 0) pieces[2].push_back(std::move(outline.sides[2]));
      if (a == 0) pieces[3].push_back(std::move(outline.sides[3]));
    }
  }

  // The window's outline is the chunks' sides on its edges, joined at the corners they share. They come from the west
  // along a row of chunks and from the north down a column, so the east side, which runs from south to north, and the
  // north side, from east to west, take theirs in reverse.
  SimplifiedSurface result;
  auto& [south, east, north, west] = result.boundary.sides;
  for (auto piece = pieces[0].begin(); piece != pieces[0].end(); ++piece) appendPiece(south, *piece);
  for (auto piece = pieces[1].rbegin(); piece != pieces[1].rend(); ++piece) appendPiece(east, *piece);
  for (auto piece = pieces[2].rbegin(); piece != pieces[2].rend(); ++piece) appendPiece(north, *piece);
  for (auto piece = pieces[3].begin(); piece != pieces[3].end(); ++piece) appendPiece(west, *piece);
  result.surface = std::move(out.mesh);
  result.gridVertices = count;
  result.gridTriangles = 2 * static_cast<std::uint64_t>(cols - 1) * static_cast<std::uint64_t>(rows - 1);
  return result;
}

}  // namespace demdl::mesh
