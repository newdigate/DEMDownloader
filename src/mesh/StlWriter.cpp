#include "mesh/StlWriter.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "mesh/AtomicFileWriter.h"

namespace demdl::mesh {

namespace {

constexpr std::size_t kHeaderBytes = 80;
constexpr std::size_t kVec3Bytes = 12;  // three float32: a normal or a vertex
constexpr std::size_t kTriangleBytes = 4 * kVec3Bytes + 2;  // a normal, three vertices and a uint16: 50
constexpr std::uint64_t kLargestVertexNumber = std::numeric_limits<std::uint32_t>::max();

// The same refusal as checkSolid's, so that a mesh no writer can place is told apart from a failing disk.
void checkCorners(const Mesh& mesh) {
  const std::size_t vertexCount = mesh.vertices.size();
  for (std::size_t k = 0; k < mesh.triangles.size(); ++k) {
    const Triangle& t = mesh.triangles[k];
    for (const std::uint32_t v : {t.a, t.b, t.c}) {
      if (v >= vertexCount) {
        throw std::invalid_argument("triangle " + std::to_string(k) + " names vertex " + std::to_string(v) +
                                    ", but the mesh has " + std::to_string(vertexCount) + " vertices");
      }
    }
  }
}

// (b - a) x (c - a) scaled to length 1, or (0, 0, 0) for a triangle with no area. The cross product is worked out in
// double from the float coordinates, the way checkSolid does, so a triangle that checkSolid finds has an area gets a
// normal here. Products of differences of floats neither overflow nor underflow a double, which leaves only a length of
// 0, or one that is not a number, to mean "no area".
std::array<float, 3> unitNormal(const Vec3& a, const Vec3& b, const Vec3& c) {
  const double ux = double{b.x} - a.x, uy = double{b.y} - a.y, uz = double{b.z} - a.z;
  const double wx = double{c.x} - a.x, wy = double{c.y} - a.y, wz = double{c.z} - a.z;
  const double nx = uy * wz - uz * wy;
  const double ny = uz * wx - ux * wz;
  const double nz = ux * wy - uy * wx;
  const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
  // Asked this way round so that a length that is not a number counts as no area.
  if (!(length > 0) || !std::isfinite(length)) return {0.0f, 0.0f, 0.0f};
  return {static_cast<float>(nx / length), static_cast<float>(ny / length), static_cast<float>(nz / length)};
}

std::uint32_t u32At(const std::uint8_t* p) {
  return std::uint32_t{p[0]} | std::uint32_t{p[1]} << 8 | std::uint32_t{p[2]} << 16 | std::uint32_t{p[3]} << 24;
}

// Three float32 as stored, in bit patterns, with minus zero changed to zero: positions are equal when these are. Not
// the floats themselves, which would make a NaN differ from itself and leave a vertex that is not a number numbered
// afresh each time it came up.
using Key = std::array<std::uint32_t, 3>;

Key keyAt(const std::uint8_t* p) {
  Key key{};
  for (std::size_t i = 0; i < key.size(); ++i) {
    const std::uint32_t bits = u32At(p + 4 * i);
    key[i] = std::bit_cast<float>(bits) == 0.0f ? 0 : bits;
  }
  return key;
}

struct KeyHash {
  std::size_t operator()(const Key& key) const noexcept {
    // The float bit patterns of round coordinates are zero in most of their low bits, so the three words are mixed
    // with splitmix64's finalizer, twice, rather than just combined.
    const auto mix = [](std::uint64_t x) {
      x ^= x >> 30;
      x *= 0xbf58476d1ce4e5b9ULL;
      x ^= x >> 27;
      x *= 0x94d049bb133111ebULL;
      return x ^ (x >> 31);
    };
    return static_cast<std::size_t>(mix(mix(key[0] | std::uint64_t{key[1]} << 32) ^ key[2]));
  }
};

[[noreturn]] void failToRead(const std::string& what, int osError = 0) {
  std::string message = what;
  if (osError != 0) message += " (" + std::generic_category().message(osError) + ")";
  throw std::runtime_error(message);
}

}  // namespace

void writeBinaryStl(const Mesh& mesh, const std::filesystem::path& path, std::string_view header) {
  if (mesh.triangles.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("an STL counts its triangles in 32 bits, and the mesh has " +
                                std::to_string(mesh.triangles.size()));
  }
  checkCorners(mesh);

  AtomicFileWriter file(path);
  file.text(header.substr(0, kHeaderBytes));
  file.zeros(kHeaderBytes - std::min(header.size(), kHeaderBytes));
  file.u32(static_cast<std::uint32_t>(mesh.triangles.size()));
  for (const Triangle& t : mesh.triangles) {
    const Vec3& a = mesh.vertices[t.a];
    const Vec3& b = mesh.vertices[t.b];
    const Vec3& c = mesh.vertices[t.c];
    for (const float component : unitNormal(a, b, c)) file.f32(component);
    for (const Vec3* corner : {&a, &b, &c}) {
      file.f32(corner->x);
      file.f32(corner->y);
      file.f32(corner->z);
    }
    file.u16(0);
  }
  file.commit();
}

Mesh readBinaryStl(const std::filesystem::path& path) {
  errno = 0;
  std::ifstream in(path, std::ios::binary);
  const int openError = errno;
  if (!in) failToRead("cannot open " + path.string(), openError);

  in.seekg(0, std::ios::end);
  const std::streamoff end = in.tellg();
  in.seekg(0, std::ios::beg);
  if (!in || end < 0) failToRead("cannot read " + path.string());
  const auto size = static_cast<std::uint64_t>(end);
  if (size < kHeaderBytes + 4) {
    failToRead(path.string() + " is not a binary STL: it has " + std::to_string(size) +
               " bytes, fewer than the 84 of a header and a triangle count");
  }

  std::array<std::uint8_t, kHeaderBytes + 4> head{};
  errno = 0;
  in.read(reinterpret_cast<char*>(head.data()), static_cast<std::streamsize>(head.size()));
  if (!in) failToRead("read failed on " + path.string(), errno);
  const std::uint32_t count = u32At(head.data() + kHeaderBytes);
  const std::uint64_t expected = head.size() + kTriangleBytes * std::uint64_t{count};
  if (size != expected) {
    failToRead(path.string() + " is not a binary STL: it declares " + std::to_string(count) +
               " triangles, which take " + std::to_string(expected) + " bytes, but the file has " +
               std::to_string(size));
  }

  Mesh mesh;
  // The size check above ties the count to the file's real length, so this reserves no more than the file holds. A
  // closed mesh has about half as many vertices as triangles; the map and the vector grow if there are more.
  mesh.triangles.reserve(count);
  mesh.vertices.reserve(count / 2 + 8);
  std::unordered_map<Key, std::uint32_t, KeyHash> numbered;
  numbered.reserve(count / 2 + 8);

  constexpr std::size_t kChunkTriangles = 4096;
  std::vector<std::uint8_t> chunk(kChunkTriangles * kTriangleBytes);
  for (std::uint64_t done = 0; done < count;) {
    const auto inChunk = static_cast<std::size_t>(std::min<std::uint64_t>(kChunkTriangles, count - done));
    errno = 0;
    in.read(reinterpret_cast<char*>(chunk.data()), static_cast<std::streamsize>(inChunk * kTriangleBytes));
    if (!in) failToRead("read failed on " + path.string(), errno);
    for (std::size_t i = 0; i < inChunk; ++i) {
      const std::uint8_t* corners = chunk.data() + i * kTriangleBytes + kVec3Bytes;  // past the stored normal
      Triangle triangle{};
      std::uint32_t* const slot[3] = {&triangle.a, &triangle.b, &triangle.c};
      for (std::size_t c = 0; c < 3; ++c) {
        const std::uint8_t* at = corners + kVec3Bytes * c;
        const std::size_t next = mesh.vertices.size();  // the number a vertex seen for the first time would get
        if (next > kLargestVertexNumber) {
          failToRead(path.string() + " has more distinct vertices than 32-bit vertex numbers count");
        }
        const auto [entry, isNew] = numbered.try_emplace(keyAt(at), static_cast<std::uint32_t>(next));
        if (isNew) {
          mesh.vertices.push_back({std::bit_cast<float>(u32At(at)), std::bit_cast<float>(u32At(at + 4)),
                                   std::bit_cast<float>(u32At(at + 8))});
        }
        *slot[c] = entry->second;
      }
      mesh.triangles.push_back(triangle);
    }
    done += inChunk;
  }
  return mesh;
}

}  // namespace demdl::mesh
