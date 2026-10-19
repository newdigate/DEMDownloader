#include "mesh/PlyWriter.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "mesh/AtomicFileWriter.h"

namespace demdl::mesh {

namespace {

std::string formatNumber(double value) {
  char text[40];
  std::snprintf(text, sizeof text, "%.15g", value);
  return text;
}

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

// The version is the project's, a PUBLIC define of demdl.
std::string header(std::size_t vertices, std::size_t faces) {
  std::string text;
  text += "ply\n";
  text += "format binary_little_endian 1.0\n";
  text += "comment dem2mesh " DL_ELEVATION_VERSION "\n";
  text += "element vertex " + std::to_string(vertices) + "\n";
  text += "property float x\n";
  text += "property float y\n";
  text += "property float z\n";
  text += "property float s\n";
  text += "property float t\n";
  text += "element face " + std::to_string(faces) + "\n";
  text += "property list uchar uint vertex_indices\n";
  text += "end_header\n";
  return text;
}

}  // namespace

void writeBinaryPly(const Mesh& surface, const std::filesystem::path& path, double widthUnits, double heightUnits) {
  const auto usable = [](double size) { return size > 0 && std::isfinite(size); };
  if (!usable(widthUnits) || !usable(heightUnits)) {
    throw std::invalid_argument("the window must be wider and taller than 0 and finite, not " +
                                formatNumber(widthUnits) + " x " + formatNumber(heightUnits));
  }
  checkCorners(surface);

  AtomicFileWriter file(path);
  file.text(header(surface.vertices.size(), surface.triangles.size()));
  for (const Vec3& v : surface.vertices) {
    file.f32(v.x);
    file.f32(v.y);
    file.f32(v.z);
    file.f32(static_cast<float>(double{v.x} / widthUnits));
    file.f32(static_cast<float>(double{v.y} / heightUnits));
  }
  for (const Triangle& t : surface.triangles) {
    file.u8(3);
    file.u32(t.a);
    file.u32(t.b);
    file.u32(t.c);
  }
  file.commit();
}

}  // namespace demdl::mesh
