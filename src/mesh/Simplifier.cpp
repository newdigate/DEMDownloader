#include "mesh/Simplifier.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mesh/PolygonTriangulation.h"
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

std::string describe(GridPoint p) { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

std::string describe(const BlockRect& rect) {
  return "the block of columns " + std::to_string(rect.firstColumn) + " to " + std::to_string(rect.lastColumn) +
         " and rows " + std::to_string(rect.firstRow) + " to " + std::to_string(rect.lastRow);
}

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

// Thrown by SurfaceBuilder when a block takes more vertices than the vertex limit leaves it; simplifyBlock takes the
// block back out of the surface and refuses it.
struct VertexLimitPassed {};

// Where the blocks' vertices go: the surface in model units. Each vertex is placed by surfaceMesh's own expressions,
// operation for operation, so that it lands on the very floats the full grid's vertex at that sample would.
//
// The vertex limit, mostVertices: a block is stopped, with VertexLimitPassed, as soon as the surface as it was before
// the block, plus the vertices the block has taken, exceeds the limit plus the samples round the block's edge. What a
// block takes is each vertex it adds and each vertex of an earlier block it shares, once each. Only the samples on the
// block's sides can be an earlier block's (blocks meet only along their sides), so the count is never more than that
// many above the surface's own size, and a block is stopped only once the surface itself is beyond the limit: never
// one whose surface would end within it.
class SurfaceBuilder {
 public:
  SurfaceBuilder(GridView grid, double widthM, double heightM, double zScale, double scale, double levelM,
                 std::uint64_t mostVertices)
      : grid_(grid),
        widthM_(widthM),
        heightM_(heightM),
        zScale_(zScale),
        scale_(scale),
        levelM_(levelM),
        mostVertices_(mostVertices) {}

  // Starts a block of width x height cells: from here what it takes is counted against the limit, and dropBlock can
  // take it back out.
  void startBlock(int width, int height) {
    blockStart_ = mesh.vertices.size();
    trianglesBefore_ = mesh.triangles.size();
    taken_ = 0;
    const std::uint64_t edge = 2 * (static_cast<std::uint64_t>(width) + static_cast<std::uint64_t>(height));
    const std::uint64_t most = std::numeric_limits<std::uint64_t>::max();
    allowed_ = mostVertices_ > most - edge ? most : mostVertices_ + edge;
  }

  // Leaves the surface as it was when the block started: its vertices and triangles, and the numbers of the outline
  // samples it added.
  void dropBlock() {
    mesh.vertices.resize(blockStart_);
    mesh.triangles.resize(trianglesBefore_);
    std::erase_if(shared_, [this](const auto& entry) { return entry.second >= blockStart_; });
  }

  // Appends the vertex of sample (column, row) and returns its number.
  std::uint32_t add(int column, int row) {
    take();
    return append(column, row);
  }

  // The number of the vertex of sample (column, row) on a block's outline: a chunk's side, or a piece's outline. Each
  // block or piece that has the sample asks for it, the two beside a side, the up to four around a corner, the two
  // pieces that touch at a vertex: the first to ask appends the vertex and the others are given its number, so that it
  // is in the output once. Only outline samples are held, so the map is no larger than the samples the sides and runs
  // keep. A vertex of an earlier block is taken by this one too; one this block has added already, where two of its
  // pieces touch, is not taken again.
  std::uint32_t addShared(int column, int row) {
    const std::uint64_t sample = static_cast<std::uint64_t>(row) * static_cast<std::uint64_t>(grid_.cols) +
                                 static_cast<std::uint64_t>(column);
    if (const auto found = shared_.find(sample); found != shared_.end()) {
      if (found->second < blockStart_) take();
      return found->second;
    }
    take();
    const std::uint32_t number = append(column, row);
    shared_.emplace(sample, number);
    return number;
  }

  Mesh mesh;

 private:
  // Counts a vertex the block takes, and stops the block when the count passes what the limit allows it.
  void take() {
    if (static_cast<std::uint64_t>(blockStart_) + ++taken_ > allowed_) throw VertexLimitPassed{};
  }

  std::uint32_t append(int column, int row) {
    const double x = column * widthM_ / (grid_.cols - 1) * scale_;
    const double y = (grid_.rows - 1 - row) * heightM_ / (grid_.rows - 1) * scale_;
    const double z = std::max(static_cast<double>(grid_.at(column, row)), levelM_) * zScale_ * scale_;
    mesh.vertices.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    return static_cast<std::uint32_t>(mesh.vertices.size() - 1);
  }

  GridView grid_;
  double widthM_, heightM_, zScale_, scale_, levelM_;
  std::unordered_map<std::uint64_t, std::uint32_t> shared_;  // by row cols + column, the outline samples' vertices
  std::uint64_t mostVertices_;
  std::size_t blockStart_ = 0, trianglesBefore_ = 0;  // the surface's vertices and triangles when the block started
  std::uint64_t taken_ = 0;                           // by the block so far
  std::uint64_t allowed_ = std::numeric_limits<std::uint64_t>::max();  // the limit plus the block's edge
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

// The samples each stretch of a grid line keeps, simplified once for every block that has it: a chunk's side, which
// the chunk beside it has too, or a run of a piece's outline, which the piece across a block's side has too (spec
// 5.3). A stretch two blocks share is the same segment of a grid line for both, known by its axis, its line and its
// range, so both are handed the same samples, which become the same vertices along their common edge. Only the
// stretches' samples are held, and what the cache hands out stays where it is while the cache lives.
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
constexpr FaceId kOutside = -1;  // what lies across an edge of the outline

// The samples of a footprint block that lie on its pieces' outlines, those the runs keep and those they leave out, in
// the block's own coordinates: a bit a sample, row by row from the block's north-west corner. A piece's faces are
// scanned with them left out, so that none of them is ever inserted from inside the piece, and the outline keeps
// exactly the samples its runs kept.
class OutlineSamples {
 public:
  explicit OutlineSamples(const BlockRect& rect)
      : stride_(static_cast<std::size_t>(rect.lastColumn - rect.firstColumn) + 1),
        bits_((stride_ * (static_cast<std::size_t>(rect.lastRow - rect.firstRow) + 1) + 63) / 64) {}

  void mark(GridPoint p) {
    const std::size_t k = index(p.x, p.y);
    bits_[k / 64] |= std::uint64_t{1} << (k % 64);
  }

  [[nodiscard]] bool has(std::int64_t x, int y) const {
    const std::size_t k = index(x, y);
    return ((bits_[k / 64] >> (k % 64)) & 1) != 0;
  }

 private:
  [[nodiscard]] std::size_t index(std::int64_t x, int y) const {
    return static_cast<std::size_t>(y) * stride_ + static_cast<std::size_t>(x);
  }

  std::size_t stride_;
  std::vector<std::uint64_t> bits_;
};

// The greedy triangulation of a block's outline and what lies inside it (spec 4.4), in the block's own coordinates: a
// vertex is a GridPoint whose x is its column and y its row, both counted from the block's north-west corner, so y
// grows to the south. orient2d takes y to grow to the north, so a face whose corners turn counter-clockwise for
// orient2d runs clockwise on the map; every face is kept that way round, and is turned back to counter-clockwise seen
// from above when it is written out. ChunkTin triangulates a rectangle's outline and PieceTin a footprint piece's; what
// follows is what the two share.
//
// Once the outline is triangulated, every face knows its worst sample, the one furthest from its plane vertically, and
// a heap of the faces gives the worst of all; it is inserted, and so on until none is further than the tolerance. A
// sample strictly inside a face splits it in three; one on an edge between two faces splits both, in four. After each
// insertion Lawson's flips restore the empty-circle property around the new vertex: an edge is flipped when the vertex
// across it lies strictly inside the circle of the face holding the new vertex (inCircle > 0, never at 0, so that four
// points on one circle are left as they are), and never an edge of the outline, which has kOutside across it. Only the
// faces an insertion makes or changes are scanned again.
//
// The samples on the outline are never scanned: the outline's stretches, a chunk's sides or a piece's runs, were
// simplified first, which put each of their samples within the tolerance of the outline's edges, and those are never
// split again, so none is ever inserted from inside, and the outline keeps exactly the samples it was given. That is
// what lets two blocks share a side. A face's own corners are not scanned either: they lie on the surface already, and
// so cannot be inserted twice, however the rounding falls.
class Tin {
 protected:
  // outline is the samples of a footprint block's outlines, which the scan leaves out, or nullptr for a window block,
  // whose sides the scan's bounds leave out. The first `tracked` vertices note the face that holds the outline edge
  // starting at them, which ChunkTin needs for its corners.
  Tin(GridView grid, BlockRect rect, double toleranceM, SurfaceBuilder& out, const OutlineSamples* outline,
      std::uint32_t tracked)
      : grid_(grid),
        rect_(rect),
        width_(rect.lastColumn - rect.firstColumn),
        height_(rect.lastRow - rect.firstRow),
        toleranceM_(toleranceM),
        out_(out),
        outline_(outline),
        tracked_(tracked) {}

  struct Vertex {
    GridPoint at;          // in the block's coordinates
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
    std::uint32_t sample;  // its row times the block's width in samples, plus its column
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

  // Adds the vertex of sample p. A sample on the block's outline may be on a side the block beside it has too, with the
  // same samples kept, on a corner up to four blocks share, or where two pieces touch, so its vertex goes into the
  // output once, with whichever block or piece reaches it first. A sample inside is the block's own.
  std::uint32_t addVertex(GridPoint p, bool onOutline) {
    const int column = rect_.firstColumn + p.x;
    const int row = rect_.firstRow + p.y;
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
  // noted for a rescan. Where an edge of the outline starts at one of the tracked vertices, the face is noted as the
  // one that holds it.
  void write(FaceId f, std::array<std::uint32_t, 3> v, std::array<FaceId, 3> across) {
    assert(orient2d(at(v[0]), at(v[1]), at(v[2])) > 0 && "a face must turn counter-clockwise for orient2d");
    Face& face = faces_[static_cast<std::size_t>(f)];
    face.v = v;
    face.across = across;
    ++face.version;
    touched_.push_back(f);
    for (std::size_t k = 0; k < 3; ++k) {
      if (across[k] == kOutside && v[k] < tracked_) outlineFrom_[v[k]] = f;
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

  // Flips edge k of face f = (x, y, p), from x = v[k] to y = v[k + 1], when the vertex q across it lies strictly inside
  // the circle through the face's corners: f and the face across become (x, q, p) and (q, y, p), in that order. Returns
  // the face across, or kOutside when there is none (an edge of the outline) or the edge is left as it is. q lying
  // strictly inside makes the four points a strictly convex quadrilateral, so both new faces have an area.
  FaceId flipIfIllegal(FaceId f, int k) {
    const Face face = faces_[static_cast<std::size_t>(f)];
    const auto corner = [&](int i) { return face.v[static_cast<std::size_t>((k + i) % 3)]; };
    const FaceId g = face.across[static_cast<std::size_t>(k)];
    if (g == kOutside) return kOutside;
    const std::uint32_t x = corner(0), y = corner(1), p = corner(2);
    const Face other = faces_[static_cast<std::size_t>(g)];
    const int j = indexOf(other, y);
    assert(other.v[static_cast<std::size_t>((j + 1) % 3)] == x && "the face across must run the edge the other way");
    const std::uint32_t q = other.v[static_cast<std::size_t>((j + 2) % 3)];
    if (inCircle(at(x), at(y), at(p), at(q)) <= 0) return kOutside;
    const FaceId acrossXQ = other.across[static_cast<std::size_t>((j + 1) % 3)];
    const FaceId acrossQY = other.across[static_cast<std::size_t>((j + 2) % 3)];
    const FaceId acrossYP = face.across[static_cast<std::size_t>((k + 1) % 3)];
    const FaceId acrossPX = face.across[static_cast<std::size_t>((k + 2) % 3)];
    write(f, {x, q, p}, {acrossXQ, g, acrossPX});
    write(g, {q, y, p}, {acrossQY, acrossYP, f});
    repoint(acrossXQ, g, f);
    repoint(acrossYP, f, g);
    return g;
  }

  // Lawson's flips around the newest vertex p. Every face waiting in pending_ has p as its third corner, so its first
  // edge, from v[0] to v[1], is the one opposite p, and only such edges can have lost the empty-circle property. A flip
  // turns (x, y, p) and (y, x, q) into (x, q, p) and (q, y, p), each with p as its third corner again, and their edges
  // opposite p wait in turn.
  void legalise() {
    while (!pending_.empty()) {
      const FaceId f = pending_.back();
      pending_.pop_back();
      const FaceId g = flipIfIllegal(f, 0);
      if (g == kOutside) continue;
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

  // Finds face's worst sample: of the samples inside it or on its edges, its corners and the block's outline left out,
  // the one furthest from its plane, vertically; of equal distances the one in the smallest row, then the smallest
  // column, as the scan goes row by row from the north and along each row from the west and gives way only to a
  // strictly larger distance. The rows are those the face's bounding box covers and each row's span comes from the
  // three edges, so no sample outside the face is looked at.
  //
  // A window block's outline is its sides, which the bounds of the rows and columns leave out. A footprint block's is
  // the samples on its pieces' outlines. The face lies inside its piece, and an outline sample is on the piece's edge,
  // so the face can have one only as a corner or on one of its own edges along the outline: at an end of a row's span
  // where such an edge crosses the row, or all along the span where it runs along the row. Trimming the span's ends of
  // outline samples, once its corners are off, therefore leaves out all of them, at two looks a row.
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
      if (outline_ != nullptr) {
        while (lo <= hi && outline_->has(lo, y)) ++lo;
        while (lo <= hi && outline_->has(hi, y)) --hi;
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
  // lies on, if any, is not on the outline, as the sample is not on the outline, so there is a face across it.
  void insert(FaceId f) {
    const Face face = faces_[static_cast<std::size_t>(f)];
    const GridPoint s = face.worst;
    assert((outline_ == nullptr || !outline_->has(s.x, s.y)) && "a sample of the outline is never inserted");
    const std::array<std::int64_t, 3> side = {orient2d(at(face.v[0]), at(face.v[1]), s),
                                              orient2d(at(face.v[1]), at(face.v[2]), s),
                                              orient2d(at(face.v[2]), at(face.v[0]), s)};
    assert(side[0] >= 0 && side[1] >= 0 && side[2] >= 0 && "the worst sample must lie in its face");
    const std::uint32_t p = addVertex(s, false);
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
    // per face the block ends with.
    std::sort(touched_.begin(), touched_.end());
    touched_.erase(std::unique(touched_.begin(), touched_.end()), touched_.end());
    for (const FaceId g : touched_) rescan(g);
    touched_.clear();
  }

  // The inside, once the outline is triangulated: every face is scanned, and the worst sample of all inserted until
  // none is beyond the tolerance.
  void refine() {
    for (FaceId f = 0; f < static_cast<FaceId>(faces_.size()); ++f) rescan(f);
    while (!heap_.empty()) {
      std::pop_heap(heap_.begin(), heap_.end(), comesAfter);
      const Candidate top = heap_.back();
      heap_.pop_back();
      if (top.version == faces_[static_cast<std::size_t>(top.face)].version) insert(top.face);
    }
  }

  // Appends the faces to the output as triangles, counter-clockwise seen from above.
  void emit() {
    for (const Face& face : faces_) {
      out_.mesh.triangles.push_back(
          {vertices_[face.v[0]].number, vertices_[face.v[2]].number, vertices_[face.v[1]].number});
    }
  }

  GridView grid_;
  BlockRect rect_;
  int width_, height_;  // the block's size in cells
  double toleranceM_;
  SurfaceBuilder& out_;
  const OutlineSamples* outline_;
  std::uint32_t tracked_;
  std::vector<Vertex> vertices_;
  std::vector<Face> faces_;
  std::vector<Candidate> heap_;
  std::vector<FaceId> pending_;  // faces whose edge opposite the newest vertex is still to be legalised
  std::vector<FaceId> touched_;  // faces written since the last rescan
  std::array<FaceId, 4> outlineFrom_ = {kOutside, kOutside, kOutside, kOutside};  // by tracked vertex, the face holding
                                                                                  // the outline edge that starts there
};

// The samples a chunk's sides keep, in the chunk's coordinates, each side from its first corner to its last as the
// outline walks it: the south side from west to east, the east side from south to north, the north side from east to
// west and the west side from north to south. The east and north sides were simplified the other way, so their samples
// are taken in reverse.
std::array<std::vector<GridPoint>, 4> sideWalks(const ChunkSides& sides, const BlockRect& rect) {
  const int width = rect.lastColumn - rect.firstColumn;
  const int height = rect.lastRow - rect.firstRow;
  std::array<std::vector<GridPoint>, 4> walks;
  for (const int column : sides.south) walks[0].push_back({column - rect.firstColumn, height});
  for (auto row = sides.east.rbegin(); row != sides.east.rend(); ++row) {
    walks[1].push_back({width, *row - rect.firstRow});
  }
  for (auto column = sides.north.rbegin(); column != sides.north.rend(); ++column) {
    walks[2].push_back({*column - rect.firstColumn, 0});
  }
  for (const int row : sides.west) walks[3].push_back({0, row - rect.firstRow});
  return walks;
}

// The greedy triangulation of a chunk, the rectangle of a window block: its outline is its four sides.
//
// The outline comes first: the four corners make two faces, cut along the diagonal from the north-west corner to the
// south-east one as a grid's cells are, and each sample a side keeps is inserted on the outline edge it falls on, which
// splits the face there in two. Then the inside, as Tin has it, with the chunk's sides left out of every scan.
class ChunkTin : private Tin {
 public:
  ChunkTin(GridView grid, BlockRect rect, double toleranceM, SurfaceBuilder& out)
      : Tin(grid, rect, toleranceM, out, nullptr, 4) {}

  // Triangulates the chunk, appends its vertices and triangles to the output and returns its outline, as vertex numbers
  // of the output. The vertices come in the order they are inserted: the corners (south-west, south-east, north-east,
  // north-west, the first vertices of the outline's sides), then the samples the sides keep, side by side in the
  // outline's order and along each side as the outline walks it, then the interior. An outline vertex that a block
  // before this one has put in the output already is not put there again.
  Boundary build(const ChunkSides& sides) {
    const std::array<std::uint32_t, 4> corner = {addVertex({0, height_}, true), addVertex({width_, height_}, true),
                                                 addVertex({width_, 0}, true), addVertex({0, 0}, true)};
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
    const std::array<std::vector<GridPoint>, 4> walks = sideWalks(sides, rect_);
    Boundary outline;
    for (std::size_t s = 0; s < 4; ++s) {
      const std::uint32_t end = corner[(s + 1) % 4];
      const std::vector<GridPoint>& walk = walks[s];
      assert(walk.size() >= 2 && same(walk.front(), at(corner[s])) && same(walk.back(), at(end)) &&
             "a side must run from its corner to the next");
      outline.sides[s].push_back(vertices_[corner[s]].number);
      for (std::size_t k = 1; k + 1 < walk.size(); ++k) {
        const std::uint32_t p = addVertex(walk[k], true);
        const FaceId f = outlineFrom_[end];
        splitEdge(f, indexOf(faces_[static_cast<std::size_t>(f)], end), p);
        legalise();
        touched_.clear();
        outline.sides[s].push_back(vertices_[p].number);
      }
      outline.sides[s].push_back(vertices_[end].number);
    }
    touched_.clear();  // the four corners' first two faces

    refine();
    emit();
    return outline;
  }
};

// The greedy triangulation of one piece of a footprint block (spec 5.2): its outline is the samples its runs keep,
// given in the order of the piece's loop and in the block's coordinates.
//
// The outline comes first, all at once: triangulatePolygon cuts it into triangles, which become the first faces with
// kOutside across every edge of the outline, and Lawson's flips over the whole of that seed make it Delaunay. Then the
// inside, as Tin has it, with every sample of the block's outlines left out of every scan: the piece's own, kept by its
// runs or not, and those of the pieces beside it, of which it can meet only a vertex where two pieces touch.
class PieceTin : private Tin {
 public:
  PieceTin(GridView grid, BlockRect rect, double toleranceM, SurfaceBuilder& out, const OutlineSamples& outline)
      : Tin(grid, rect, toleranceM, out, &outline, 0) {}

  // Triangulates the piece whose outline is `kept`, appends its vertices and triangles to the output and returns its
  // outline as vertex numbers of the output, with the same points. The outline's vertices come first, in its order,
  // those a block or piece before this one has put in the output already left out; then the inside's, in the order
  // they are inserted.
  PieceOutline build(const std::vector<GridPoint>& kept) {
    PieceOutline outline;
    std::vector<GridPoint> seenFromAbove;  // y north, as triangulatePolygon and orient2d take it
    seenFromAbove.reserve(kept.size());
    for (const GridPoint p : kept) {
      outline.vertices.push_back(vertices_[addVertex(p, true)].number);
      outline.points.push_back(p);
      seenFromAbove.push_back({p.x, -p.y});
    }
    seed(triangulatePolygon(seenFromAbove));
    legaliseAll();
    touched_.clear();

    refine();
    emit();
    return outline;
  }

 private:
  // Makes the first faces from triangles of the outline's vertices that turn counter-clockwise seen from above, and so
  // clockwise for orient2d: each face takes its triangle's corners the other way round. Across each edge is the face
  // that has the same edge the other way, found by sorting the edges by their ends, or kOutside for an edge of the
  // outline, which only one triangle has.
  void seed(const std::vector<std::array<std::uint32_t, 3>>& triangles) {
    struct HalfEdge {
      std::uint64_t ends;  // the lower vertex, then the higher
      FaceId face;
      std::size_t k;
    };
    assert(faces_.empty() && "the seed's faces are numbered as its triangles");
    std::vector<std::array<std::uint32_t, 3>> corners;
    std::vector<HalfEdge> halves;
    corners.reserve(triangles.size());
    halves.reserve(3 * triangles.size());
    for (const auto& [a, b, c] : triangles) {
      const auto f = static_cast<FaceId>(corners.size());
      const std::array<std::uint32_t, 3> v = {a, c, b};
      corners.push_back(v);
      for (std::size_t k = 0; k < 3; ++k) {
        const std::uint32_t p = v[k], q = v[(k + 1) % 3];
        halves.push_back({(std::uint64_t{std::min(p, q)} << 32) | std::max(p, q), f, k});
      }
    }
    std::sort(halves.begin(), halves.end(), [](const HalfEdge& p, const HalfEdge& q) { return p.ends < q.ends; });
    std::vector<std::array<FaceId, 3>> across(corners.size(), {kOutside, kOutside, kOutside});
    for (std::size_t h = 0; h + 1 < halves.size(); ++h) {
      if (halves[h].ends != halves[h + 1].ends) continue;
      assert((h + 2 == halves.size() || halves[h + 2].ends != halves[h].ends) && "an edge has two faces at most");
      across[static_cast<std::size_t>(halves[h].face)][halves[h].k] = halves[h + 1].face;
      across[static_cast<std::size_t>(halves[h + 1].face)][halves[h + 1].k] = halves[h].face;
      ++h;
    }
    for (std::size_t f = 0; f < corners.size(); ++f) write(newFace(), corners[f], across[f]);
  }

  // Lawson's flips over the whole seed, which no insertion made: every edge between two faces is tested, and each
  // flip tests the four edges round its quadrilateral again, until no vertex lies strictly inside the circle of the
  // face across an edge from it. An edge, once legal, can lose that only when one of its two faces changes, and every
  // face a flip changes has its edges tested again, so none is missed; each flip makes the triangulation strictly more
  // Delaunay, so the flips end. The outline's edges are never flipped.
  void legaliseAll() {
    std::vector<std::pair<FaceId, int>> edges;
    for (FaceId f = 0; f < static_cast<FaceId>(faces_.size()); ++f) {
      for (int k = 0; k < 3; ++k) {
        if (faces_[static_cast<std::size_t>(f)].across[static_cast<std::size_t>(k)] > f) edges.push_back({f, k});
      }
    }
    while (!edges.empty()) {
      const auto [f, k] = edges.back();
      edges.pop_back();
      const FaceId g = flipIfIllegal(f, k);
      touched_.clear();
      if (g == kOutside) continue;
      // f is (x, q, p) now and g (q, y, p): the edges from x to q, p to x, q to y and y to p.
      edges.insert(edges.end(), {{f, 0}, {f, 2}, {g, 0}, {g, 1}});
    }
  }
};

// The four sides of a window block, as simplifyGrid takes a chunk's: each from the cache, so that a side two blocks
// share is simplified once.
ChunkSides chunkSides(SideCache& cache, const BlockRect& rect) {
  return {cache.alongRow(rect.firstRow, rect.firstColumn, rect.lastColumn),
          cache.alongRow(rect.lastRow, rect.firstColumn, rect.lastColumn),
          cache.alongColumn(rect.firstColumn, rect.firstRow, rect.lastRow),
          cache.alongColumn(rect.lastColumn, rect.firstRow, rect.lastRow)};
}

// A window block's outline as a footprint piece has it: from the block's north-west corner down its west side, then
// along its south side, up its east side and back along its north side, as the loop of a piece that is the whole block
// runs, each side's last vertex left to the next. boundary is ChunkTin's, and walks the sides' samples as it walks
// them.
PieceOutline rectangleOutline(const Boundary& boundary, const std::array<std::vector<GridPoint>, 4>& walks) {
  PieceOutline outline;
  for (const std::size_t side : {std::size_t{3}, std::size_t{0}, std::size_t{1}, std::size_t{2}}) {
    assert(boundary.sides[side].size() == walks[side].size() && "a side's vertices are its walk's samples");
    for (std::size_t k = 0; k + 1 < walks[side].size(); ++k) {
      outline.vertices.push_back(boundary.sides[side][k]);
      outline.points.push_back(walks[side][k]);
    }
  }
  return outline;
}

// Whether a footprint block is one piece that is the whole block, its runs the block's four sides whole: a loop whose
// four runs start at the block's north-west, south-west, south-east and north-east corners. The cells across each side
// are then all in or all out, the block beside it has the side whole or not at all, and the block is simplified as a
// window block is, from the same sides, to the same surface.
bool isWholeBlock(const BlockFootprint& footprint, const BlockRect& rect) {
  if (footprint.pieces.size() != 1 || footprint.pieces[0].runStarts.size() != 4) return false;
  const Piece& piece = footprint.pieces[0];
  const std::array<GridPoint, 4> corners = {GridPoint{rect.firstColumn, rect.firstRow},
                                            GridPoint{rect.firstColumn, rect.lastRow},
                                            GridPoint{rect.lastColumn, rect.lastRow},
                                            GridPoint{rect.lastColumn, rect.firstRow}};
  for (std::size_t k = 0; k < 4; ++k) {
    const GridPoint p = piece.loop[piece.runStarts[k]];
    if (p.x != corners[k].x || p.y != corners[k].y) return false;
  }
  return true;
}

// The samples a piece's runs keep, in the block's coordinates and in the loop's order from its first sample (spec 5.2,
// 5.3). Each run is simplified by the cache in the fixed direction, along a row from west to east and along a column
// from north to south, and its samples are taken in reverse where the loop walks it the other way; so a run two blocks'
// pieces share, walked one way by one and the other way by the other, is simplified once and keeps the same samples for
// both. A run's last sample is the next run's first, and the last run's the loop's first, so each comes once.
std::vector<GridPoint> keptOutline(const Piece& piece, const BlockRect& rect, SideCache& cache) {
  std::vector<GridPoint> kept;
  const std::vector<std::uint32_t>& starts = piece.runStarts;
  for (std::size_t r = 0; r < starts.size(); ++r) {
    const GridPoint from = piece.loop[starts[r]];
    const GridPoint to = piece.loop[r + 1 < starts.size() ? starts[r + 1] : 0];
    const bool alongRow = from.y == to.y;
    const std::span<const int> along = alongRow
                                           ? cache.alongRow(from.y, std::min(from.x, to.x), std::max(from.x, to.x))
                                           : cache.alongColumn(from.x, std::min(from.y, to.y), std::max(from.y, to.y));
    const auto add = [&](int k) {
      kept.push_back(alongRow ? GridPoint{k - rect.firstColumn, from.y - rect.firstRow}
                              : GridPoint{from.x - rect.firstColumn, k - rect.firstRow});
    };
    if (alongRow ? from.x < to.x : from.y < to.y) {
      for (std::size_t k = 0; k + 1 < along.size(); ++k) add(along[k]);
    } else {
      for (std::size_t k = along.size() - 1; k > 0; --k) add(along[k]);
    }
  }
  return kept;
}

// What is wrong with a footprint piece as a piece of the block rect, or "" when nothing is. Its loop must hold four
// samples or more, all in the block and none twice, each one cell from the next along a grid line, and turn
// counter-clockwise seen from above; its runs must start at 0 and rise within the loop, each run straight. A walk along
// grid lines a cell a step can meet itself only at a sample, so with no sample twice the loop is a simple polygon, and
// triangulatePolygon cuts its kept samples without fail.
std::string pieceProblem(const Piece& piece, const BlockRect& rect) {
  const std::vector<GridPoint>& loop = piece.loop;
  const std::size_t n = loop.size();
  if (n < 4) return "has " + std::to_string(n) + " samples, too few for a loop";
  for (const GridPoint& p : loop) {
    if (p.x < rect.firstColumn || p.x > rect.lastColumn || p.y < rect.firstRow || p.y > rect.lastRow) {
      return "has the sample " + describe(p) + ", outside " + describe(rect);
    }
  }
  const std::vector<std::uint32_t>& starts = piece.runStarts;
  bool rising = !starts.empty() && starts.front() == 0 && starts.back() < n;
  for (std::size_t r = 1; r < starts.size() && rising; ++r) rising = starts[r - 1] < starts[r];
  if (!rising) {
    std::string list;
    for (const std::uint32_t k : starts) list += (list.empty() ? "" : ", ") + std::to_string(k);
    return "has its runs start at " + list + ": they must start at 0 and rise, within its loop of " +
           std::to_string(n) + " samples";
  }
  for (std::size_t k = 0; k < n; ++k) {
    const GridPoint p = loop[k], q = loop[(k + 1) % n];
    if (std::abs(q.x - p.x) + std::abs(q.y - p.y) != 1) {
      return "steps from " + describe(p) + " to " + describe(q) + ", not one cell along a grid line";
    }
  }
  std::size_t run = 0;
  for (std::size_t k = 1; k < n; ++k) {
    if (run + 1 < starts.size() && k == starts[run + 1]) {
      ++run;
      continue;
    }
    const GridPoint a = loop[k - 1], b = loop[k], c = loop[(k + 1) % n];
    if (b.x - a.x != c.x - b.x || b.y - a.y != c.y - b.y) {
      return "has a run from " + describe(loop[starts[run]]) + " that turns at " + describe(b);
    }
  }
  std::vector<std::pair<int, int>> samples;
  samples.reserve(n);
  for (const GridPoint& p : loop) samples.push_back({p.y, p.x});
  std::sort(samples.begin(), samples.end());
  const auto twice = std::adjacent_find(samples.begin(), samples.end());
  if (twice != samples.end()) return "meets itself at " + describe(GridPoint{twice->second, twice->first});
  // Twice the area seen from above, y north: the shoelace sum on the block's own columns and rows, whose terms stay far
  // inside 64 bits, with its sign turned because the rows run south.
  std::int64_t twiceArea = 0;
  for (std::size_t k = 0; k < n; ++k) {
    const std::int64_t x0 = loop[k].x - rect.firstColumn, y0 = loop[k].y - rect.firstRow;
    const std::int64_t x1 = loop[(k + 1) % n].x - rect.firstColumn, y1 = loop[(k + 1) % n].y - rect.firstRow;
    twiceArea -= x0 * y1 - x1 * y0;
  }
  if (twiceArea <= 0) return "runs clockwise seen from above";
  return "";
}

// Throws unless rect is a block of the grid the simplifier can take: inside the grid, at least one cell each way, and
// no more than kMostChunkCells, the largest side the predicates are exact for (spec 4.5).
void checkBlock(const BlockRect& rect, int cols, int rows) {
  if (rect.firstColumn < 0 || rect.firstRow < 0 || rect.lastColumn > cols - 1 || rect.lastRow > rows - 1) {
    throw std::invalid_argument(describe(rect) + " is not inside the grid of " + gridSize(cols, rows) + " vertices");
  }
  if (rect.lastColumn <= rect.firstColumn || rect.lastRow <= rect.firstRow) {
    throw std::invalid_argument(describe(rect) + " has no cell");
  }
  const auto tooLarge = [&](int cells, const char* way) {
    return std::invalid_argument(describe(rect) + " is " + std::to_string(cells) + " cells " + way +
                                 "; a block is at most " + std::to_string(kMostChunkCells) + " cells a side");
  };
  if (rect.lastColumn - rect.firstColumn > kMostChunkCells) throw tooLarge(rect.lastColumn - rect.firstColumn, "wide");
  if (rect.lastRow - rect.firstRow > kMostChunkCells) throw tooLarge(rect.lastRow - rect.firstRow, "tall");
}

// The checks of the grid and the tolerance, the first of simplifyGrid's and a BlockSimplifier's, in the order the
// header of simplifyGrid gives them. heights is not looked at.
void checkGridAndTolerance(const std::vector<float>& heights, int cols, int rows, double toleranceM) {
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
}

// The rest of their checks: surfaceMesh's of the frame, with its messages, and then every height finite.
void checkFrameAndHeights(GridView grid, double widthM, double heightM, double zScale, double scale, double levelM) {
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
  for (int row = 0; row < grid.rows; ++row) {
    for (int column = 0; column < grid.cols; ++column) {
      if (!std::isfinite(grid.at(column, row))) {
        throw std::invalid_argument("the height at column " + std::to_string(column) + ", row " + std::to_string(row) +
                                    " is not finite");
      }
    }
  }
}

// Appends a piece of the window's outline, the side of one chunk on the window's edge, to the side of the window it
// lies on. A piece after the first starts at the corner where the side so far ends, which the two chunks share and
// which is listed once.
void appendPiece(std::vector<std::uint32_t>& side, const std::vector<std::uint32_t>& piece) {
  assert(!piece.empty() && (side.empty() || side.back() == piece.front()) && "neighbouring chunks share a corner");
  side.insert(side.end(), piece.begin() + (side.empty() ? 0 : 1), piece.end());
}

}  // namespace

// What a BlockSimplifier holds between blocks: the grid, the surface so far, and the stretches of grid lines the blocks
// have simplified, with the vertex numbers of the samples they keep.
struct BlockSimplifier::State {
  State(GridView grid, double toleranceM, double widthM, double heightM, double zScale, double scale, double levelM,
        std::uint64_t mostVertices)
      : grid(grid),
        toleranceM(toleranceM),
        mostVertices(mostVertices),
        out(grid, widthM, heightM, zScale, scale, levelM, mostVertices),
        sides(grid, toleranceM) {}

  GridView grid;
  double toleranceM;
  std::uint64_t mostVertices;
  SurfaceBuilder out;
  SideCache sides;
  bool taken = false;  // whether takeSurface has moved the surface out
};

BlockSimplifier::BlockSimplifier(const std::vector<float>& heights, int cols, int rows, double toleranceM,
                                 double widthM, double heightM, double zScale, double scale, double levelM,
                                 std::uint64_t mostVertices) {
  checkGridAndTolerance(heights, cols, rows, toleranceM);
  const GridView grid{heights.data(), cols, rows};
  checkFrameAndHeights(grid, widthM, heightM, zScale, scale, levelM);
  state_ = std::make_unique<State>(grid, toleranceM, widthM, heightM, zScale, scale, levelM, mostVertices);
}

BlockSimplifier::~BlockSimplifier() = default;

BlockResult BlockSimplifier::simplifyBlock(BlockRect rect, const BlockFootprint* footprint) {
  State& s = *state_;
  if (s.taken) throw std::logic_error("simplifyBlock after takeSurface: the surface has been taken");
  checkBlock(rect, s.grid.cols, s.grid.rows);
  if (footprint != nullptr) {
    for (const Piece& piece : footprint->pieces) {
      const std::string problem = pieceProblem(piece, rect);
      if (problem.empty()) continue;
      throw std::invalid_argument(
          (piece.loop.empty() ? std::string("the footprint has a piece that")
                              : "the footprint's piece whose loop starts at " + describe(piece.loop.front())) +
          " " + problem);
    }
  }

  BlockResult result;
  result.firstTriangle = s.out.mesh.triangles.size();
  s.out.startBlock(rect.lastColumn - rect.firstColumn, rect.lastRow - rect.firstRow);
  try {
    if (footprint == nullptr) {
      result.rectangle = ChunkTin(s.grid, rect, s.toleranceM, s.out).build(chunkSides(s.sides, rect));
    } else if (isWholeBlock(*footprint, rect)) {
      const ChunkSides sides = chunkSides(s.sides, rect);
      const Boundary rectangle = ChunkTin(s.grid, rect, s.toleranceM, s.out).build(sides);
      result.pieces.push_back(rectangleOutline(rectangle, sideWalks(sides, rect)));
    } else if (!footprint->pieces.empty()) {
      // Every outline sample of the block is marked before the first piece is triangulated, so that a vertex where two
      // pieces touch is left out of the second's scans as well as the first's.
      OutlineSamples outline(rect);
      for (const Piece& piece : footprint->pieces) {
        for (const GridPoint& p : piece.loop) outline.mark({p.x - rect.firstColumn, p.y - rect.firstRow});
      }
      for (const Piece& piece : footprint->pieces) {
        const std::vector<GridPoint> kept = keptOutline(piece, rect, s.sides);
        result.pieces.push_back(PieceTin(s.grid, rect, s.toleranceM, s.out, outline).build(kept));
      }
    }
  } catch (const VertexLimitPassed&) {
    s.out.dropBlock();
    const std::string most = std::to_string(s.mostVertices);
    throw std::runtime_error("the simplified mesh is more than " + most + " vertices; the limit is " + most +
                             ": raise --tolerance, or use --grid or crop");
  }
  result.triangles = s.out.mesh.triangles.size() - result.firstTriangle;
  return result;
}

const Mesh& BlockSimplifier::surface() const noexcept { return state_->out.mesh; }

Mesh BlockSimplifier::takeSurface() {
  state_->taken = true;
  return std::move(state_->out.mesh);
}

SimplifiedSurface simplifyGrid(const std::vector<float>& heights, int cols, int rows, double toleranceM, double widthM,
                               double heightM, double zScale, double scale, double levelM, int chunkCells,
                               std::uint64_t mostVertices) {
  checkGridAndTolerance(heights, cols, rows, toleranceM);
  if (chunkCells < 1 || chunkCells > kMostChunkCells) {
    throw std::invalid_argument("a chunk must be 1 to " + std::to_string(kMostChunkCells) + " cells a side, not " +
                                std::to_string(chunkCells));
  }
  BlockSimplifier simplifier(heights, cols, rows, toleranceM, widthM, heightM, zScale, scale, levelM, mostVertices);

  // The lattice (spec 4.5): chunks of chunkCells cells a side from the north-west corner, those in the last column and
  // the last row of chunks cut short, taken row by row from the north and along each row from the west, each a window
  // block of the simplifier. A chunk's sides come from the simplifier's cache and its outline's vertices from its
  // shared numbers, so the chunks either side of a seam meet on the same vertices, each in the output once. Counted so
  // that no int overflows, whatever the grid.
  const int across = (cols - 2) / chunkCells + 1;
  const int down = (rows - 2) / chunkCells + 1;
  std::array<std::vector<std::vector<std::uint32_t>>, 4> pieces;  // the window's sides, by chunk as they come
  for (int b = 0; b < down; ++b) {
    const int firstRow = b * chunkCells;
    const int lastRow = firstRow + std::min(chunkCells, rows - 1 - firstRow);
    for (int a = 0; a < across; ++a) {
      const int firstColumn = a * chunkCells;
      const int lastColumn = firstColumn + std::min(chunkCells, cols - 1 - firstColumn);
      Boundary outline = *simplifier.simplifyBlock(BlockRect{firstColumn, firstRow, lastColumn, lastRow}, nullptr)
                              .rectangle;
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
  result.surface = simplifier.takeSurface();
  result.gridVertices = static_cast<std::uint64_t>(cols) * static_cast<std::uint64_t>(rows);
  result.gridTriangles = 2 * static_cast<std::uint64_t>(cols - 1) * static_cast<std::uint64_t>(rows - 1);
  return result;
}

}  // namespace demdl::mesh
