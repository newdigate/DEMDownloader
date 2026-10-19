#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <signal.h>
#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include "TempDir.h"
#include "mesh/AtomicFileWriter.h"
#include "mesh/PlyWriter.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using demdl::mesh::AtomicFileWriter;
using demdl::mesh::checkSolid;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
using demdl::mesh::plyFileBytes;
using demdl::mesh::readBinaryStl;
using demdl::mesh::solidMesh;
using demdl::mesh::solidTriangleCount;
using demdl::mesh::stlFileBytes;
using demdl::mesh::surfaceMesh;
using demdl::mesh::Triangle;
using demdl::mesh::Vec3;
using demdl::mesh::writeBinaryPly;
using demdl::mesh::writeBinaryStl;

// The layouts are pinned here as literal bytes, and everything else is decoded byte by byte, least significant first,
// so that neither the host's byte order nor a change to a writer can move a file and its test together.

namespace {

using Bytes = std::vector<unsigned char>;
using Grid = std::vector<float>;

constexpr std::size_t kStlHeaderBytes = 80;
constexpr std::size_t kStlFirstTriangle = 84;  // the header, then a uint32 count
constexpr std::size_t kStlTriangleBytes = 50;  // 12 floats (the normal and three vertices) and a uint16
constexpr std::size_t kPlyVertexBytes = 20;    // x y z s t, as floats
constexpr std::size_t kPlyFaceBytes = 13;      // a uchar 3 and three uint32

Bytes readFile(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  REQUIRE(in.is_open());
  return Bytes(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void writeFile(const std::filesystem::path& file, const std::string& content) {
  std::ofstream out(file, std::ios::binary);
  out << content;
  out.close();
  REQUIRE(out.good());
}

std::filesystem::path tempOf(const std::filesystem::path& file) { return file.string() + ".tmp"; }

std::uint32_t u32At(const Bytes& bytes, std::size_t at) {
  // Not an assertion: the big files read a few hundred thousand of these.
  if (at + 4 > bytes.size()) {
    throw std::out_of_range("4 bytes at " + std::to_string(at) + " are past the end of a file of " +
                            std::to_string(bytes.size()) + " bytes");
  }
  return std::uint32_t{bytes[at]} | std::uint32_t{bytes[at + 1]} << 8 | std::uint32_t{bytes[at + 2]} << 16 |
         std::uint32_t{bytes[at + 3]} << 24;
}

float f32At(const Bytes& bytes, std::size_t at) { return std::bit_cast<float>(u32At(bytes, at)); }

// "0c 00 00 00": count bytes from at, in the file's order.
std::string hex(const Bytes& bytes, std::size_t at, std::size_t count) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string text;
  for (std::size_t i = at; i < at + count && i < bytes.size(); ++i) {
    if (!text.empty()) text += ' ';
    text += kDigits[bytes[i] >> 4];
    text += kDigits[bytes[i] & 15];
  }
  return text;
}

// The box of test_SolidMesh.cpp: a 2 x 2 grid of heights 10 over 100 m by 50 m, no exaggeration or scaling, level 0,
// closed down to z = -5: 8 vertices, 12 triangles, volume 100 x 50 x 15 = 75000. Surface vertex 0 is the north-west
// corner, 1 the north-east, 2 the south-west and 3 the south-east.
Mesh boxSurface() { return surfaceMesh(Grid(4, 10.0f), 2, 2, 100, 50, 1, 1, 0); }
Mesh box() { return solidMesh(boxSurface(), 2, 2, -5.0f); }

// A rough surface of cols x rows vertices over 1000 m by 800 m with heights from 10 to 110 m, drawn from std::mt19937,
// whose output the standard fixes. At 200 x 160 its STL and PLY run to several times the writers' staging buffer, so a
// record straddles a flush somewhere.
Mesh roughSurface(int cols, int rows) {
  std::mt19937 rng(7);
  Grid heights(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
  for (float& h : heights) h = 10.0f + static_cast<float>(rng() % 1001) / 10.0f;
  return surfaceMesh(heights, cols, rows, 1000, 800, 1, 1, 0);
}

struct StlRecord {
  std::array<float, 3> normal;
  std::array<std::array<float, 3>, 3> corner;
  std::uint16_t attribute;
};

StlRecord stlRecord(const Bytes& bytes, std::size_t k) {
  std::size_t at = kStlFirstTriangle + k * kStlTriangleBytes;
  StlRecord record{};
  for (float& c : record.normal) {
    c = f32At(bytes, at);
    at += 4;
  }
  for (auto& corner : record.corner) {
    for (float& c : corner) {
      c = f32At(bytes, at);
      at += 4;
    }
  }
  record.attribute = static_cast<std::uint16_t>(bytes[at] | bytes[at + 1] << 8);
  return record;
}

std::array<float, 3> xyz(const Vec3& v) { return {v.x, v.y, v.z}; }

// (b - a) x (c - a) scaled to length 1, in double: the normal a viewer would work out from the corners as written.
std::array<double, 3> unitNormal(const std::array<std::array<float, 3>, 3>& c) {
  const std::array<double, 3> u = {double{c[1][0]} - c[0][0], double{c[1][1]} - c[0][1], double{c[1][2]} - c[0][2]};
  const std::array<double, 3> w = {double{c[2][0]} - c[0][0], double{c[2][1]} - c[0][1], double{c[2][2]} - c[0][2]};
  std::array<double, 3> n = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
  const double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  for (double& x : n) x /= length;
  return n;
}

double lengthOf(const std::array<float, 3>& v) {
  return std::sqrt(double{v[0]} * v[0] + double{v[1]} * v[1] + double{v[2]} * v[2]);
}

struct PlyVertex {
  float x, y, z, s, t;
};

// The PLY header, line by line as the format and Blender's importer want it, for a mesh of the given counts.
std::string plyHeader(std::size_t vertices, std::size_t faces) {
  std::string header;
  header += "ply\n";
  header += "format binary_little_endian 1.0\n";
  header += "comment dem2mesh " DL_ELEVATION_VERSION "\n";
  header += "element vertex " + std::to_string(vertices) + "\n";
  header += "property float x\n";
  header += "property float y\n";
  header += "property float z\n";
  header += "property float s\n";
  header += "property float t\n";
  header += "element face " + std::to_string(faces) + "\n";
  header += "property list uchar uint vertex_indices\n";
  header += "end_header\n";
  return header;
}

PlyVertex plyVertex(const Bytes& bytes, std::size_t firstVertex, std::size_t k) {
  const std::size_t at = firstVertex + k * kPlyVertexBytes;
  return {f32At(bytes, at), f32At(bytes, at + 4), f32At(bytes, at + 8), f32At(bytes, at + 12), f32At(bytes, at + 16)};
}

// Lowers this process's file-size limit, with SIGXFSZ ignored so that a write past the limit fails with EFBIG instead
// of killing the process. Restores both on destruction, however the scope ends. (As in test_DemRaster.cpp.)
class FileSizeLimit {
 public:
  explicit FileSizeLimit(std::uintmax_t bytes) {
    struct sigaction ignoreAction {};
    ignoreAction.sa_handler = SIG_IGN;
    sigemptyset(&ignoreAction.sa_mask);
    ignored_ = sigaction(SIGXFSZ, &ignoreAction, &previousAction_) == 0;
    if (ignored_ && getrlimit(RLIMIT_FSIZE, &previousLimit_) == 0) {
      rlimit lowered = previousLimit_;
      lowered.rlim_cur = static_cast<rlim_t>(bytes);
      lowered_ = setrlimit(RLIMIT_FSIZE, &lowered) == 0;
    }
  }
  ~FileSizeLimit() {
    if (lowered_) setrlimit(RLIMIT_FSIZE, &previousLimit_);
    if (ignored_) sigaction(SIGXFSZ, &previousAction_, nullptr);
  }
  FileSizeLimit(const FileSizeLimit&) = delete;
  FileSizeLimit& operator=(const FileSizeLimit&) = delete;

  bool active() const { return lowered_; }

 private:
  struct sigaction previousAction_ {};
  rlimit previousLimit_{};
  bool ignored_ = false;
  bool lowered_ = false;
};

struct LimitedWrite {
  bool limited = false;  // the limit was in force during the write
  std::string error;     // empty when the write succeeded
};

// Runs write under a file-size limit. Nothing is reported while the limit is in force, so no test output can be cut
// short by it.
template <typename Write>
LimitedWrite writeUnderLimit(std::uintmax_t bytes, Write write) {
  LimitedWrite result;
  const FileSizeLimit limit(bytes);
  result.limited = limit.active();
  try {
    write();
  } catch (const std::runtime_error& e) {
    result.error = e.what();
  }
  return result;
}

}  // namespace

TEST_CASE("the STL of the 2 x 2 solid has the size, the count and the byte layout of the format", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "box.stl";
  writeBinaryStl(box(), file, "box");

  CHECK(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
  const Bytes bytes = readFile(file);
  REQUIRE(bytes.size() == 84 + 50 * 12);
  CHECK(hex(bytes, 80, 4) == "0c 00 00 00");  // 12 triangles, least significant byte first

  // The first triangle is the surface's: north-west, south-west, south-east, which faces up. Its normal (0, 0, 1) has
  // a z of 1.0f = 0x3F800000, which goes out as 00 00 80 3f, and zeros that may be of either sign. The corners: 10.0f
  // is 0x41200000, 50.0f is 0x42480000 and 100.0f is 0x42C80000.
  CHECK(f32At(bytes, 84) == 0.0f);
  CHECK(f32At(bytes, 88) == 0.0f);
  CHECK(hex(bytes, 92, 4) == "00 00 80 3f");
  CHECK(hex(bytes, 96, 36) ==
        "00 00 00 00 00 00 48 42 00 00 20 41"    // north-west (0, 50, 10)
        " 00 00 00 00 00 00 00 00 00 00 20 41"   // south-west (0, 0, 10)
        " 00 00 c8 42 00 00 00 00 00 00 20 41");  // south-east (100, 0, 10)
  CHECK(hex(bytes, 132, 2) == "00 00");           // the attribute byte count
}

TEST_CASE("every STL triangle carries its corners, an outward unit normal and a zero attribute", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "box.stl";
  const Mesh solid = box();
  writeBinaryStl(solid, file, "box");
  const Bytes bytes = readFile(file);
  REQUIRE(bytes.size() == 84 + 50 * 12);

  // The box's faces are all flat and square to the axes, so each normal is one of the six unit axis vectors, exactly,
  // and the solid has two triangles facing each way: up on the surface, down on the base, and out on each wall.
  std::array<std::array<float, 3>, 6> axes = {{{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}}};
  std::array<int, 6> facing{};
  for (std::size_t k = 0; k < solid.triangles.size(); ++k) {
    INFO("triangle " << k);
    const StlRecord record = stlRecord(bytes, k);
    const Triangle& t = solid.triangles[k];
    CHECK(record.corner[0] == xyz(solid.vertices[t.a]));
    CHECK(record.corner[1] == xyz(solid.vertices[t.b]));
    CHECK(record.corner[2] == xyz(solid.vertices[t.c]));
    CHECK(record.attribute == 0);
    CHECK_THAT(lengthOf(record.normal), WithinAbs(1.0, 1e-6));
    for (std::size_t axis = 0; axis < axes.size(); ++axis) {
      if (record.normal == axes[axis]) ++facing[axis];
    }
  }
  CHECK(facing == std::array<int, 6>{2, 2, 2, 2, 2, 2});
}

TEST_CASE("the STL normal of a sloping triangle is its cross product scaled to length 1", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "slope.stl";
  const Mesh surface = surfaceMesh(Grid{10, 20, 30, 50, 40, 70}, 3, 2, 30, 20, 1, 1, 0);
  const Mesh solid = solidMesh(surface, 3, 2, -5.0f);
  writeBinaryStl(solid, file, "slope");
  const Bytes bytes = readFile(file);
  REQUIRE(bytes.size() == 84 + 50 * solid.triangles.size());
  for (std::size_t k = 0; k < solid.triangles.size(); ++k) {
    INFO("triangle " << k);
    const StlRecord record = stlRecord(bytes, k);
    const std::array<double, 3> expected = unitNormal(record.corner);
    CHECK_THAT(lengthOf(record.normal), WithinAbs(1.0, 1e-6));
    for (std::size_t i = 0; i < 3; ++i) CHECK_THAT(double{record.normal[i]}, WithinAbs(expected[i], 1e-6));
  }
}

TEST_CASE("an STL triangle without area gets the zero vector for a normal", "[writers]") {
  TempDir dir;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // Three points in line, a point twice, and a triangle with a coordinate that is not a number: no direction to give.
  const Mesh degenerate{{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 1, 0}, {nan, 1, 0}},
                        {{0, 1, 2}, {0, 1, 1}, {0, 1, 3}, {0, 1, 4}}};
  const auto file = dir.path / "flat.stl";
  writeBinaryStl(degenerate, file, "flat");
  const Bytes bytes = readFile(file);
  REQUIRE(bytes.size() == 84 + 50 * 4);
  for (std::size_t k : {0u, 1u, 3u}) {
    INFO("triangle " << k);
    const StlRecord record = stlRecord(bytes, k);
    CHECK(record.normal == std::array<float, 3>{0, 0, 0});  // not NaN either
  }
  // The triangle with area beside them keeps its normal.
  CHECK(stlRecord(bytes, 2).normal == std::array<float, 3>{0, 0, 1});
}

TEST_CASE("an STL header longer than 80 bytes is cut and a shorter one padded with zeros", "[writers]") {
  TempDir dir;
  const Mesh solid = box();
  const auto headerWritten = [&](const std::string& text) {
    const auto file = dir.path / "header.stl";
    writeBinaryStl(solid, file, text);
    const Bytes bytes = readFile(file);
    REQUIRE(bytes.size() == 84 + 50 * 12);
    CHECK(hex(bytes, 80, 4) == "0c 00 00 00");  // the count follows the 80 bytes whatever the header's length
    return std::string(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(kStlHeaderBytes));
  };

  std::string longer;
  for (int i = 0; i < 100; ++i) longer += static_cast<char>('a' + i % 26);
  CHECK(headerWritten(longer) == longer.substr(0, 80));
  CHECK(headerWritten("dem2mesh test") == std::string("dem2mesh test") + std::string(67, '\0'));
  CHECK(headerWritten("") == std::string(80, '\0'));
  CHECK(headerWritten(std::string(80, 'z')) == std::string(80, 'z'));
}

TEST_CASE("readBinaryStl gives back the 2 x 2 solid with its vertices merged", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "box.stl";
  const Mesh solid = box();
  writeBinaryStl(solid, file, "box");

  const Mesh back = readBinaryStl(file);
  CHECK(back.vertices.size() == 8);
  CHECK(back.triangles.size() == 12);
  const MeshCheck check = checkSolid(back);
  CHECK(check.edgesChecked);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK_THAT(check.signedVolume, WithinAbs(75000.0, 1e-3));
  CHECK(check.ok());
  // Every triangle has its corners where the original had them.
  for (std::size_t k = 0; k < solid.triangles.size(); ++k) {
    CHECK(xyz(back.vertices[back.triangles[k].a]) == xyz(solid.vertices[solid.triangles[k].a]));
    CHECK(xyz(back.vertices[back.triangles[k].b]) == xyz(solid.vertices[solid.triangles[k].b]));
    CHECK(xyz(back.vertices[back.triangles[k].c]) == xyz(solid.vertices[solid.triangles[k].c]));
  }
}

TEST_CASE("readBinaryStl numbers vertices in order of first appearance and merges equal positions, minus zero too",
          "[writers]") {
  TempDir dir;
  const auto file = dir.path / "quad.stl";
  // Two triangles of one square written the way an STL does, each with its own copies of the shared corners; the copy
  // of (1, 0, 0) has a minus zero for its y.
  const Mesh unmerged{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, -0.0f, 0}, {1, 1, 0}, {0, 1, 0}}, {{0, 1, 2}, {3, 4, 5}}};
  writeBinaryStl(unmerged, file, "quad");
  const Mesh back = readBinaryStl(file);
  REQUIRE(back.vertices.size() == 4);
  CHECK(xyz(back.vertices[0]) == std::array<float, 3>{0, 0, 0});
  CHECK(xyz(back.vertices[1]) == std::array<float, 3>{1, 0, 0});
  CHECK(xyz(back.vertices[2]) == std::array<float, 3>{0, 1, 0});
  CHECK(xyz(back.vertices[3]) == std::array<float, 3>{1, 1, 0});
  REQUIRE(back.triangles.size() == 2);
  CHECK(std::make_tuple(back.triangles[0].a, back.triangles[0].b, back.triangles[0].c) == std::make_tuple(0u, 1u, 2u));
  CHECK(std::make_tuple(back.triangles[1].a, back.triangles[1].b, back.triangles[1].c) == std::make_tuple(1u, 3u, 2u));
}

TEST_CASE("readBinaryStl refuses what is not a binary STL, naming the file", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "bad.stl";

  CHECK_THROWS_AS(readBinaryStl(dir.path / "absent.stl"), std::runtime_error);
  CHECK_THROWS_WITH(readBinaryStl(dir.path), ContainsSubstring(dir.path.string()));  // a directory
  CHECK_THROWS_WITH(readBinaryStl(dir.path / "absent.stl"),
                    ContainsSubstring((dir.path / "absent.stl").string()) &&
                        ContainsSubstring(std::generic_category().message(ENOENT)));

  writeFile(file, "solid tiny\nendsolid tiny\n");  // an ASCII STL, and shorter than a header
  CHECK_THROWS_AS(readBinaryStl(file), std::runtime_error);
  CHECK_THROWS_WITH(readBinaryStl(file), ContainsSubstring(file.string()));

  // A real file one byte short of its count, and one byte over.
  const auto good = dir.path / "good.stl";
  writeBinaryStl(box(), good, "box");
  std::string content;
  {
    const Bytes bytes = readFile(good);
    content.assign(bytes.begin(), bytes.end());
  }
  writeFile(file, content.substr(0, content.size() - 1));
  CHECK_THROWS_WITH(readBinaryStl(file), ContainsSubstring(file.string()) && ContainsSubstring("12 triangles"));
  writeFile(file, content + "x");
  CHECK_THROWS_WITH(readBinaryStl(file), ContainsSubstring(file.string()) && ContainsSubstring("12 triangles"));
}

TEST_CASE("a large rough solid survives the STL and its staging buffer's flushes", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "rough.stl";
  const Mesh solid = solidMesh(roughSurface(200, 160), 200, 160, -5.0f);
  const std::uint64_t count = solidTriangleCount(200, 160);
  REQUIRE(solid.triangles.size() == count);
  writeBinaryStl(solid, file, "rough");

  // 3.2 MB, past three of the buffer's flushes, so that a record straddles one.
  CHECK(std::filesystem::file_size(file) == 84 + 50 * count);
  CHECK(std::filesystem::file_size(file) > 3 * AtomicFileWriter::kBufferBytes);
  const Mesh back = readBinaryStl(file);
  CHECK(back.vertices.size() == solid.vertices.size());
  REQUIRE(back.triangles.size() == solid.triangles.size());
  bool same = true;
  for (std::size_t k = 0; k < solid.triangles.size() && same; ++k) {
    const Triangle& a = solid.triangles[k];
    const Triangle& b = back.triangles[k];
    same = xyz(back.vertices[b.a]) == xyz(solid.vertices[a.a]) && xyz(back.vertices[b.b]) == xyz(solid.vertices[a.b]) &&
           xyz(back.vertices[b.c]) == xyz(solid.vertices[a.c]);
    if (!same) FAIL("triangle " << k << " differs after the round trip");
  }
  const MeshCheck before = checkSolid(solid);
  const MeshCheck after = checkSolid(back);
  CHECK(after.ok());
  CHECK(after.nonManifoldEdges == 0);
  CHECK_THAT(after.signedVolume, WithinRel(before.signedVolume, 1e-9));
}

TEST_CASE("the PLY of the 2 x 2 surface has the header, the vertices with s and t, and the faces", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "box.ply";
  const Mesh surface = boxSurface();
  writeBinaryPly(surface, file, 100, 50);

  CHECK(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
  const Bytes bytes = readFile(file);
  const std::string header = plyHeader(4, 2);
  REQUIRE(bytes.size() == header.size() + 4 * kPlyVertexBytes + 2 * kPlyFaceBytes);
  CHECK(std::string(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(header.size())) == header);

  const std::size_t vertices = header.size();
  // Vertex 1 is the north-east corner (100, 50, 10), at the far corner of the texture: s = 1, t = 1. Vertex 2 is the
  // south-west corner (0, 0, 10), at its origin. 1.0f is 0x3F800000.
  const PlyVertex northEast = plyVertex(bytes, vertices, 1);
  CHECK(std::make_tuple(northEast.x, northEast.y, northEast.z, northEast.s, northEast.t) ==
        std::make_tuple(100.0f, 50.0f, 10.0f, 1.0f, 1.0f));
  CHECK(hex(bytes, vertices + 20, 20) == "00 00 c8 42 00 00 48 42 00 00 20 41 00 00 80 3f 00 00 80 3f");
  const PlyVertex southWest = plyVertex(bytes, vertices, 2);
  CHECK(std::make_tuple(southWest.x, southWest.y, southWest.z, southWest.s, southWest.t) ==
        std::make_tuple(0.0f, 0.0f, 10.0f, 0.0f, 0.0f));
  const PlyVertex northWest = plyVertex(bytes, vertices, 0);
  CHECK(std::make_tuple(northWest.x, northWest.y, northWest.z, northWest.s, northWest.t) ==
        std::make_tuple(0.0f, 50.0f, 10.0f, 0.0f, 1.0f));
  const PlyVertex southEast = plyVertex(bytes, vertices, 3);
  CHECK(std::make_tuple(southEast.x, southEast.y, southEast.z, southEast.s, southEast.t) ==
        std::make_tuple(100.0f, 0.0f, 10.0f, 1.0f, 0.0f));

  // The faces follow, in the surface's order: a count of 3 as one byte, then three vertex numbers as uint32.
  const std::size_t faces = vertices + 4 * kPlyVertexBytes;
  CHECK(hex(bytes, faces, 13) == "03 00 00 00 00 02 00 00 00 03 00 00 00");  // (0, 2, 3)
  CHECK(hex(bytes, faces + 13, 13) == "03 00 00 00 00 03 00 00 00 01 00 00 00");  // (0, 3, 1)
}

TEST_CASE("PLY s and t are x and y over the window size given, not over the mesh's own extent", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "uv.ply";
  const Mesh surface = surfaceMesh(Grid{1, 2, 3, 4, 5, 6}, 3, 2, 30, 20, 1, 1, 0);
  writeBinaryPly(surface, file, 120, 8);
  const Bytes bytes = readFile(file);
  const std::string header = plyHeader(6, 4);
  REQUIRE(bytes.size() == header.size() + 6 * kPlyVertexBytes + 4 * kPlyFaceBytes);
  for (std::size_t k = 0; k < surface.vertices.size(); ++k) {
    INFO("vertex " << k);
    const PlyVertex v = plyVertex(bytes, header.size(), k);
    CHECK(v.x == surface.vertices[k].x);
    CHECK(v.y == surface.vertices[k].y);
    CHECK(v.z == surface.vertices[k].z);
    CHECK_THAT(double{v.s}, WithinAbs(double{surface.vertices[k].x} / 120.0, 1e-7));
    CHECK_THAT(double{v.t}, WithinAbs(double{surface.vertices[k].y} / 8.0, 1e-7));
  }
  // The north-west corner, y = 20, over a height of 8: t is y / height and nothing more, so it may pass 1.
  CHECK(plyVertex(bytes, header.size(), 0).t == 2.5f);
}

TEST_CASE("a large PLY keeps every vertex and face across the staging buffer's flushes", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "rough.ply";
  const Mesh surface = roughSurface(200, 160);
  writeBinaryPly(surface, file, 1000, 800);

  const Bytes bytes = readFile(file);
  const std::string header = plyHeader(surface.vertices.size(), surface.triangles.size());
  REQUIRE(bytes.size() == header.size() + kPlyVertexBytes * surface.vertices.size() +
                              kPlyFaceBytes * surface.triangles.size());
  CHECK(bytes.size() > AtomicFileWriter::kBufferBytes);
  CHECK(std::string(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(header.size())) == header);
  bool same = true;
  for (std::size_t k = 0; k < surface.vertices.size() && same; ++k) {
    const PlyVertex v = plyVertex(bytes, header.size(), k);
    same = v.x == surface.vertices[k].x && v.y == surface.vertices[k].y && v.z == surface.vertices[k].z &&
           v.s == static_cast<float>(double{surface.vertices[k].x} / 1000.0) &&
           v.t == static_cast<float>(double{surface.vertices[k].y} / 800.0);
    if (!same) FAIL("vertex " << k << " differs in the file");
  }
  const std::size_t faces = header.size() + kPlyVertexBytes * surface.vertices.size();
  for (std::size_t k = 0; k < surface.triangles.size() && same; ++k) {
    const std::size_t at = faces + k * kPlyFaceBytes;
    same = bytes[at] == 3 && u32At(bytes, at + 1) == surface.triangles[k].a &&
           u32At(bytes, at + 5) == surface.triangles[k].b && u32At(bytes, at + 9) == surface.triangles[k].c;
    if (!same) FAIL("face " << k << " differs in the file");
  }
}

TEST_CASE("a mesh with nothing in it writes a valid empty file", "[writers]") {
  TempDir dir;
  const Mesh empty;
  writeBinaryStl(empty, dir.path / "empty.stl", "nothing");
  const Bytes stl = readFile(dir.path / "empty.stl");
  REQUIRE(stl.size() == 84);
  CHECK(hex(stl, 80, 4) == "00 00 00 00");
  const Mesh back = readBinaryStl(dir.path / "empty.stl");
  CHECK(back.vertices.empty());
  CHECK(back.triangles.empty());

  writeBinaryPly(empty, dir.path / "empty.ply", 10, 10);
  const Bytes ply = readFile(dir.path / "empty.ply");
  const std::string header = plyHeader(0, 0);
  CHECK(std::string(ply.begin(), ply.end()) == header);
}

TEST_CASE("the file sizes the writers give beforehand are the sizes of the files they write", "[writers]") {
  TempDir dir;
  const auto stl = dir.path / "sized.stl";
  const auto ply = dir.path / "sized.ply";
  const auto writtenSizesMatch = [&](const Mesh& surface, const Mesh& solid) {
    writeBinaryPly(surface, ply, 1000, 800);
    writeBinaryStl(solid, stl, "sizes");
    CHECK(plyFileBytes(surface.vertices.size(), surface.triangles.size()) == std::filesystem::file_size(ply));
    CHECK(stlFileBytes(solid.triangles.size()) == std::filesystem::file_size(stl));
  };
  writtenSizesMatch(boxSurface(), box());
  const Mesh rough = roughSurface(200, 160);  // files of several staging buffers
  writtenSizesMatch(rough, solidMesh(rough, 200, 160, -5.0f));
  writtenSizesMatch(Mesh{}, Mesh{});

  // At the 25,000,000-vertex limit, a 5000 x 5000 grid, which no test writes and the dry run states unwritten: sizes past
  // 32 bits, and a solid of 2 x 4999 x 4999 + 2 x 10000 + 2 triangles.
  CHECK(stlFileBytes(solidTriangleCount(5000, 5000)) == 84 + 50 * std::uint64_t{50'000'004});
  CHECK(plyFileBytes(25'000'000, 49'980'002) ==
        plyHeader(25'000'000, 49'980'002).size() + 20 * std::uint64_t{25'000'000} + 13 * std::uint64_t{49'980'002});
}

TEST_CASE("AtomicFileWriter writes numbers least significant byte first and splits long runs across its flushes",
          "[writers]") {
  TempDir dir;
  const auto file = dir.path / "pieces.bin";
  constexpr std::size_t kBuffer = AtomicFileWriter::kBufferBytes;
  // Neither run starts at the top of a buffer, and each is longer than the whole buffer, so each goes out in pieces.
  const std::string words(kBuffer + 100, 'w');
  const std::size_t zeros = 2 * kBuffer + 3;
  {
    AtomicFileWriter out(file);
    out.u8(7);
    out.text(words);
    out.u16(0x0201);
    out.zeros(zeros);
    out.u32(0x04030201);
    out.f32(1.0f);
    out.commit();
  }
  const Bytes bytes = readFile(file);
  REQUIRE(bytes.size() == 1 + words.size() + 2 + zeros + 4 + 4);
  CHECK(bytes[0] == 7);
  const auto wordsEnd = bytes.begin() + 1 + static_cast<std::ptrdiff_t>(words.size());
  CHECK(std::all_of(bytes.begin() + 1, wordsEnd, [](unsigned char c) { return c == 'w'; }));
  CHECK(hex(bytes, 1 + words.size(), 2) == "01 02");
  const auto zerosEnd = wordsEnd + 2 + static_cast<std::ptrdiff_t>(zeros);
  CHECK(std::all_of(wordsEnd + 2, zerosEnd, [](unsigned char c) { return c == 0; }));
  CHECK(hex(bytes, 1 + words.size() + 2 + zeros, 8) == "01 02 03 04 00 00 80 3f");
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
}

TEST_CASE("an AtomicFileWriter dropped before commit leaves neither its file nor its temp file", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "dropped.bin";
  {
    AtomicFileWriter out(file);
    out.u32(1);
    CHECK(std::filesystem::exists(tempOf(file)));  // the .tmp stands from the start
  }
  CHECK_FALSE(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
}

TEST_CASE("a file in a missing directory throws with the path and the reason and leaves nothing", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "absent" / "out";
  const std::string reason = std::generic_category().message(ENOENT);

  SECTION("STL") {
    CHECK_THROWS_AS(writeBinaryStl(box(), file, "box"), std::runtime_error);
    CHECK_THROWS_WITH(writeBinaryStl(box(), file, "box"),
                      ContainsSubstring(file.string()) && ContainsSubstring(reason));
  }
  SECTION("PLY") {
    CHECK_THROWS_AS(writeBinaryPly(boxSurface(), file, 100, 50), std::runtime_error);
    CHECK_THROWS_WITH(writeBinaryPly(boxSurface(), file, 100, 50),
                      ContainsSubstring(file.string()) && ContainsSubstring(reason));
  }
  CHECK_FALSE(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
  CHECK_FALSE(std::filesystem::exists(dir.path / "absent"));
}

TEST_CASE("a write that runs out of room throws with the reason, keeps the old file and removes the temp file",
          "[writers]") {
  TempDir dir;
  const auto file = dir.path / "out";
  writeFile(file, "the previous file");

  SECTION("an STL that does not fit, small enough to go out in the last flush") {
    const Mesh solid = box();
    const LimitedWrite attempt = writeUnderLimit(300, [&] { writeBinaryStl(solid, file, "box"); });
    REQUIRE(attempt.limited);
    CHECK_THAT(attempt.error, ContainsSubstring(tempOf(file).string()));
    CHECK_THAT(attempt.error, ContainsSubstring("File too large"));
  }
  SECTION("an STL that does not fit, bigger than the staging buffer") {
    const Mesh solid = solidMesh(roughSurface(200, 160), 200, 160, -5.0f);
    const LimitedWrite attempt = writeUnderLimit(300, [&] { writeBinaryStl(solid, file, "rough"); });
    REQUIRE(attempt.limited);
    CHECK_THAT(attempt.error, ContainsSubstring(tempOf(file).string()));
    CHECK_THAT(attempt.error, ContainsSubstring("File too large"));
  }
  SECTION("an STL one byte short of the room it needs") {
    const Mesh solid = box();
    const LimitedWrite attempt = writeUnderLimit(84 + 50 * 12 - 1, [&] { writeBinaryStl(solid, file, "box"); });
    REQUIRE(attempt.limited);
    CHECK_THAT(attempt.error, ContainsSubstring(tempOf(file).string()));
    CHECK_THAT(attempt.error, ContainsSubstring("File too large"));
  }
  SECTION("a PLY that does not fit, bigger than the staging buffer") {
    const Mesh surface = roughSurface(200, 160);
    const LimitedWrite attempt = writeUnderLimit(300, [&] { writeBinaryPly(surface, file, 1000, 800); });
    REQUIRE(attempt.limited);
    CHECK_THAT(attempt.error, ContainsSubstring(tempOf(file).string()));
    CHECK_THAT(attempt.error, ContainsSubstring("File too large"));
  }
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
  const Bytes kept = readFile(file);
  CHECK(std::string(kept.begin(), kept.end()) == "the previous file");
}

TEST_CASE("a destination that cannot be replaced throws and leaves no temp file", "[writers]") {
  TempDir dir;
  // A file cannot be renamed over a directory, so the write completes and the rename fails.
  const auto file = dir.path / "out";
  std::filesystem::create_directory(file);

  SECTION("STL") { CHECK_THROWS_WITH(writeBinaryStl(box(), file, "box"), ContainsSubstring("cannot rename")); }
  SECTION("PLY") { CHECK_THROWS_WITH(writeBinaryPly(boxSurface(), file, 100, 50), ContainsSubstring("cannot rename")); }
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
  CHECK(std::filesystem::is_directory(file));
}

TEST_CASE("a finished write replaces an existing file and a stale temp file", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "out";
  writeFile(file, "the previous file, longer than nothing");
  writeFile(tempOf(file), "left by a run that died");

  SECTION("STL") {
    writeBinaryStl(box(), file, "box");
    CHECK(std::filesystem::file_size(file) == 84 + 50 * 12);
  }
  SECTION("PLY") {
    writeBinaryPly(boxSurface(), file, 100, 50);
    CHECK(std::filesystem::file_size(file) == plyHeader(4, 2).size() + 4 * kPlyVertexBytes + 2 * kPlyFaceBytes);
  }
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
}

TEST_CASE("a triangle that names a vertex the mesh lacks is refused and leaves no file", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "out";

  SECTION("STL") {
    Mesh solid = box();
    solid.triangles[3].b = 8;  // the box has vertices 0 to 7
    CHECK_THROWS_AS(writeBinaryStl(solid, file, "box"), std::invalid_argument);
    CHECK_THROWS_WITH(writeBinaryStl(solid, file, "box"), ContainsSubstring("triangle 3 names vertex 8"));
  }
  SECTION("PLY") {
    Mesh surface = boxSurface();
    surface.triangles[1].c = 4;  // the surface has vertices 0 to 3
    CHECK_THROWS_AS(writeBinaryPly(surface, file, 100, 50), std::invalid_argument);
    CHECK_THROWS_WITH(writeBinaryPly(surface, file, 100, 50), ContainsSubstring("triangle 1 names vertex 4"));
  }
  CHECK_FALSE(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
}

TEST_CASE("a PLY window that is not above 0 and finite is refused and leaves no file", "[writers]") {
  TempDir dir;
  const auto file = dir.path / "out.ply";
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const Mesh surface = boxSurface();
  for (const auto& [width, height] : std::vector<std::pair<double, double>>{
           {0, 50}, {100, 0}, {-100, 50}, {100, -50}, {nan, 50}, {100, nan}, {inf, 50}, {100, -inf}}) {
    INFO("window " << width << " x " << height);
    CHECK_THROWS_AS(writeBinaryPly(surface, file, width, height), std::invalid_argument);
  }
  CHECK_THROWS_WITH(writeBinaryPly(surface, file, 0, 50),
                    ContainsSubstring("the window must be wider and taller than 0"));
  CHECK_FALSE(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(tempOf(file)));
}
