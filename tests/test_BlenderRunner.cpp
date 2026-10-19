#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "FakeBlender.h"
#include "SpawnedProcess.h"  // readFile, fileHolds, writeFile
#include "TempDir.h"
#include "mesh/BlenderRunner.h"
#include "mesh/BlenderScenePy.h"
#include "mesh/PlyWriter.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"

using Catch::Matchers::ContainsSubstring;
using demdl::mesh::BlenderJob;
using demdl::mesh::BlenderResult;
using demdl::mesh::blenderStartProblem;
using demdl::mesh::findBlender;
using demdl::mesh::findOnPath;
using demdl::mesh::kBlenderAppExecutable;
using demdl::mesh::kBlenderScenePy;
using demdl::mesh::Mesh;
using demdl::mesh::namedBlender;
using demdl::mesh::runBlender;

namespace {

using Path = std::filesystem::path;

// A job in dir. The numbers have no short decimal form, so that an argument that rounds them is caught, and the low end
// is negative, which an argument parser can take for an option.
BlenderJob exampleJob(const Path& dir) {
  return {dir / "model.ply", dir / "model.stl", dir / "model.blend", "model", 74.5 / 3, 76.1, -0.1 / 3, 21.69, 0.02};
}

// Whether this machine has the Blender app where findBlender looks for it, before PATH.
bool appInstalled() {
  std::error_code ignored;
  return std::filesystem::exists(kBlenderAppExecutable, ignored);
}

// A file at path that the owner may run.
void writeExecutable(const Path& path) {
  writeFile(path, "#!/bin/sh\nexit 0\n");
  std::filesystem::permissions(path, std::filesystem::perms::owner_all);
}

// Makes dir the working directory until it goes.
class ScopedCurrentPath {
 public:
  explicit ScopedCurrentPath(const Path& dir) : old_(std::filesystem::current_path()) {
    std::filesystem::current_path(dir);
  }
  ~ScopedCurrentPath() {
    std::error_code ignored;
    std::filesystem::current_path(old_, ignored);
  }
  ScopedCurrentPath(const ScopedCurrentPath&) = delete;
  ScopedCurrentPath& operator=(const ScopedCurrentPath&) = delete;

 private:
  Path old_;
};

}  // namespace

TEST_CASE("findBlender takes the --blender path as given, an empty one included", "[blender]") {
  const ScopedEnv blender("BLENDER", "/elsewhere/blender");
  CHECK(findBlender(Path("/opt/blender/blender")) == Path("/opt/blender/blender"));
  CHECK(findBlender(Path("relative/blender")) == Path("relative/blender"));
  // Never looked at here: a path that names nothing is for the run to refuse, naming it.
  CHECK(findBlender(Path("/nonexistent/blender")) == Path("/nonexistent/blender"));
  // An empty --blender is a bad path the user gave, not "no path", so it must not fall through to another Blender.
  const std::optional<Path> empty = findBlender(Path(""));
  REQUIRE(empty.has_value());
  CHECK(empty->empty());
}

TEST_CASE("findBlender takes BLENDER when no path is given, unless BLENDER is empty", "[blender]") {
  const TempDir dir;
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv path("PATH", dir.path.string());  // a blender on PATH too, which BLENDER comes before
  {
    const ScopedEnv blender("BLENDER", fake.executable.string());
    CHECK(findBlender(std::nullopt) == fake.executable);
  }
  {
    const ScopedEnv blender("BLENDER", "/nonexistent/blender");
    CHECK(findBlender(std::nullopt) == Path("/nonexistent/blender"));  // as given, like the flag
  }
  {
    const ScopedEnv blender("BLENDER", "");
    CHECK(findBlender(std::nullopt) == (appInstalled() ? Path(kBlenderAppExecutable) : fake.executable));
  }
}

TEST_CASE("findBlender then takes the Blender app, then the first blender on PATH, and else finds none", "[blender]") {
  const TempDir dir;
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv blender("BLENDER", std::nullopt);
  // Where the app is installed it comes first and PATH is never searched; findOnPath's own test covers PATH there.
  {
    const ScopedEnv path("PATH", dir.path.string());
    CHECK(findBlender(std::nullopt) == (appInstalled() ? Path(kBlenderAppExecutable) : fake.executable));
  }
  for (const std::optional<std::string>& value : {std::optional<std::string>(""), std::optional<std::string>()}) {
    CAPTURE(value.has_value());
    const ScopedEnv path("PATH", value);
    if (appInstalled()) {
      CHECK(findBlender(std::nullopt) == Path(kBlenderAppExecutable));
    } else {
      CHECK_FALSE(findBlender(std::nullopt).has_value());
    }
  }
}

TEST_CASE("findOnPath takes the first executable file of the name in the list's order", "[blender]") {
  const TempDir dir;
  const Path nothing = dir.path / "nothing";      // no blender
  const Path plain = dir.path / "plain";          // a blender that may not be run
  const Path folder = dir.path / "folder";        // a directory called blender
  const Path first = dir.path / "first";          // the one to find
  const Path second = dir.path / "second";        // another, later in the list
  for (const Path& d : {nothing, plain, folder, first, second}) REQUIRE(std::filesystem::create_directory(d));
  writeFile(plain / "blender", "#!/bin/sh\nexit 0\n");
  std::filesystem::permissions(plain / "blender", std::filesystem::perms::owner_read | std::filesystem::perms::owner_write);
  REQUIRE(std::filesystem::create_directory(folder / "blender"));
  writeExecutable(first / "blender");
  writeExecutable(second / "blender");

  const std::string list = ":" + nothing.string() + ":" + plain.string() + ":" + folder.string() + "::" +
                           first.string() + ":" + second.string() + ":";
  CHECK(findOnPath("blender", list) == first / "blender");
  CHECK(findOnPath("blender", second.string()) == second / "blender");
  CHECK_FALSE(findOnPath("blender", nothing.string() + ":" + plain.string() + ":" + folder.string()).has_value());
  CHECK_FALSE(findOnPath("blender", "").has_value());
  CHECK_FALSE(findOnPath("blender", dir.path.string() + "/missing").has_value());
  // An empty entry is not read as the working directory, as a shell would: a blender beside the user is never run
  // unasked.
  const ScopedCurrentPath here(first);
  CHECK_FALSE(findOnPath("blender", ":").has_value());
  CHECK_FALSE(findOnPath("blender", "").has_value());
}

TEST_CASE("namedBlender is the --blender path, else BLENDER unless it is empty, else none", "[blender]") {
  {
    const ScopedEnv blender("BLENDER", "/elsewhere/blender");
    CHECK(namedBlender(Path("/opt/blender/blender")) == Path("/opt/blender/blender"));
    CHECK(namedBlender(Path("blender")) == Path("blender"));
    CHECK(namedBlender(std::nullopt) == Path("/elsewhere/blender"));
  }
  for (const std::optional<std::string>& value : {std::optional<std::string>(""), std::optional<std::string>()}) {
    CAPTURE(value.has_value());
    const ScopedEnv blender("BLENDER", value);
    CHECK(namedBlender(Path("/opt/blender/blender")) == Path("/opt/blender/blender"));
    CHECK_FALSE(namedBlender(std::nullopt).has_value());
  }
}

TEST_CASE("blenderStartProblem says why a named Blender cannot be started, and nothing when it can", "[blender]") {
  const TempDir dir;
  const Path executable = dir.path / "blender";
  writeExecutable(executable);
  const Path plain = dir.path / "plain";  // a file that may not be run
  writeFile(plain, "#!/bin/sh\nexit 0\n");
  std::filesystem::permissions(plain, std::filesystem::perms::owner_read | std::filesystem::perms::owner_write);
  const Path folder = dir.path / "folder";
  REQUIRE(std::filesystem::create_directory(folder));
  const Path app = dir.path / "Blender.app";
  REQUIRE(std::filesystem::create_directory(app));
  const Path link = dir.path / "link";
  std::filesystem::create_symlink(executable, link);
  const Path dangling = dir.path / "dangling";
  std::filesystem::create_symlink(dir.path / "missing", dangling);

  CHECK_FALSE(blenderStartProblem(executable).has_value());
  CHECK_FALSE(blenderStartProblem(link).has_value());  // a link to an executable file counts
  CHECK(blenderStartProblem(dir.path / "missing") == "does not exist");
  CHECK(blenderStartProblem(dir.path / "missing" / "blender") == "does not exist");
  CHECK(blenderStartProblem(dangling) == "does not exist");
  CHECK(blenderStartProblem(plain) == "is not executable");
  CHECK(blenderStartProblem(folder) == "is not a regular file");
  // The app is a directory; what runs is inside it. A trailing slash, as a shell's completion adds, changes nothing.
  const std::string inside = "is the app bundle; its executable is " + app.string() + "/Contents/MacOS/Blender";
  CHECK(blenderStartProblem(app) == inside);
  CHECK(blenderStartProblem(app.string() + "/") == inside);

  // A name without a slash is looked for on PATH, as a shell would look it up, and only an executable file counts.
  {
    const ScopedEnv path("PATH", dir.path.string());
    CHECK_FALSE(blenderStartProblem("blender").has_value());
    CHECK(blenderStartProblem("plain") == "not found on PATH");
    CHECK(blenderStartProblem("folder") == "not found on PATH");
    CHECK(blenderStartProblem("missing") == "not found on PATH");
  }
  {
    const ScopedEnv path("PATH", std::nullopt);
    CHECK(blenderStartProblem("blender") == "not found on PATH");
  }
  // A relative path with a slash in it is taken from the working directory.
  const ScopedCurrentPath here(dir.path);
  CHECK_FALSE(blenderStartProblem("./blender").has_value());
  CHECK(blenderStartProblem("./folder") == "is not a regular file");
}

TEST_CASE("runBlender runs Blender in the background on the embedded script with the job after --", "[blender]") {
  const TempDir dir;
  const TempDir scripts;  // the temporary directory while it runs, so that the script's file can be seen to go
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv tmpdir("TMPDIR", scripts.path.string());
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  const BlenderJob job = exampleJob(dir.path);

  const BlenderResult result = runBlender(fake.executable, job);
  INFO("output:\n" << result.output);
  CHECK(result.exitCode == 0);
  // Both streams in one string, in the order they were written, and nothing of the runner's own when all went well.
  CHECK(result.output == "fake blender: to stdout\nfake blender: to stderr\n");

  const std::vector<std::string> args = fake.recordedArguments();
  REQUIRE(args.size() == 14);
  CHECK(args[0] == "--background");
  CHECK(args[1] == "--factory-startup");
  CHECK(args[2] == "--python");
  CHECK(args[4] == "--");
  CHECK(args[5] == "--ply=" + job.ply.string());
  CHECK(args[6] == "--stl=" + job.stl.string());
  CHECK(args[7] == "--blend=" + job.blend.string());
  CHECK(args[8] == "--name=" + job.name);
  // Every digit a double needs, so that the script reads back the very numbers.
  CHECK(numberIn(args[9], "--width") == job.widthUnits);
  CHECK(numberIn(args[10], "--height") == job.heightUnits);
  CHECK(numberIn(args[11], "--zmin") == job.minZ);
  CHECK(numberIn(args[12], "--zmax") == job.maxZ);
  CHECK(numberIn(args[13], "--scale") == job.scale);

  // The script was a file in the temporary directory while Blender ran, the embedded script whole, and it is gone.
  const Path script = args[3];
  CHECK(script.extension() == ".py");
  CHECK(std::filesystem::equivalent(script.parent_path(), scripts.path));
  CHECK(fileHolds(fake.scriptCopy, std::string(kBlenderScenePy)));
  CHECK_FALSE(std::filesystem::exists(script));
  CHECK(std::filesystem::is_empty(scripts.path));
}

TEST_CASE("runBlender gives back Blender's exit code and output when it fails, and removes the script", "[blender]") {
  const TempDir dir;
  const TempDir scripts;
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv tmpdir("TMPDIR", scripts.path.string());
  const ScopedEnv exit("FAKE_EXIT", "3");
  const BlenderResult result = runBlender(fake.executable, exampleJob(dir.path));
  INFO("output:\n" << result.output);
  CHECK(result.exitCode == 3);
  CHECK(result.output == "fake blender: to stdout\nfake blender: to stderr\n");
  CHECK(std::filesystem::exists(fake.scriptCopy));  // it had its script
  CHECK(std::filesystem::is_empty(scripts.path));
}

TEST_CASE("runBlender gives exit 127 and names an executable it cannot start, leaving no script behind", "[blender]") {
  const TempDir dir;
  const TempDir scripts;
  const ScopedEnv tmpdir("TMPDIR", scripts.path.string());
  const Path notExecutable = dir.path / "blender.sh";
  writeFile(notExecutable, "#!/bin/sh\nexit 0\n");
  for (const Path& blender : {Path("/nonexistent/blender"), Path(""), dir.path, notExecutable}) {
    CAPTURE(blender);
    const BlenderResult result = runBlender(blender, exampleJob(dir.path));
    INFO("output:\n" << result.output);
    CHECK(result.exitCode == 127);
    CHECK_THAT(result.output, ContainsSubstring("cannot start '" + blender.string() + "': "));
    CHECK(std::filesystem::is_empty(scripts.path));
  }
}

TEST_CASE("runBlender reports a Blender that a signal ended", "[blender]") {
  const TempDir dir;
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv signal("FAKE_SIGNAL", "9");
  const BlenderResult result = runBlender(fake.executable, exampleJob(dir.path));
  INFO("output:\n" << result.output);
  CHECK(result.exitCode == 128 + 9);  // as a shell gives it
  CHECK_THAT(result.output, ContainsSubstring("fake blender: to stdout\nfake blender: to stderr\n"));
  CHECK_THAT(result.output, ContainsSubstring("'" + fake.executable.string() + "' was ended by signal 9"));
}

TEST_CASE("runBlender looks a name without a slash up on PATH, as a shell would", "[blender]") {
  const TempDir dir;
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  const ScopedEnv path("PATH", dir.path.string());
  const BlenderResult result = runBlender("blender", exampleJob(dir.path));  // as from --blender blender
  INFO("output:\n" << result.output);
  CHECK(result.exitCode == 0);
  CHECK(fake.recordedArguments().size() == 14);
}

TEST_CASE("runBlender gives Blender absolute paths, which it resolves the same wherever it runs", "[blender]") {
  const TempDir dir;
  const FakeBlender fake = writeFakeBlender(dir.path);
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  BlenderJob job = exampleJob(dir.path);
  job.ply = "out/model.ply";
  job.stl = "out/model.stl";
  job.blend = "out/model.blend";
  REQUIRE(runBlender(fake.executable, job).exitCode == 0);
  const std::vector<std::string> args = fake.recordedArguments();
  REQUIRE(args.size() == 14);
  const Path here = std::filesystem::current_path();
  CHECK(args[5] == "--ply=" + (here / "out/model.ply").string());
  CHECK(args[6] == "--stl=" + (here / "out/model.stl").string());
  CHECK(args[7] == "--blend=" + (here / "out/model.blend").string());
}

TEST_CASE("the embedded scene script imports the PLY and checks the STL", "[blender]") {
  const std::string script(kBlenderScenePy);
  CHECK_THAT(script, ContainsSubstring("ply_import"));
  CHECK_THAT(script, ContainsSubstring("stl_import"));
  CHECK_THAT(script, ContainsSubstring("save_as_mainfile"));
  CHECK_THAT(script, ContainsSubstring("stl check:"));
}

TEST_CASE("the installed Blender sets the units from the scale and keeps the terrain in place, and its check fails "
          "each flawed solid",
          "[blender][slow]") {
  const std::optional<Path> blender = findBlender(std::nullopt);
  if (!blender) SKIP("Blender not installed");
  const TempDir dir;
  // A 2 x 2 grid, flat at 10 m, 100 m by 50 m: the surface is two triangles and the solid a box of twelve. Each run
  // below takes Blender's start-up, six seconds on the owner's machine, so the cases share four runs, each with its
  // own scale for the scene's units.
  const Mesh surface = demdl::mesh::surfaceMesh({10.0f, 10.0f, 10.0f, 10.0f}, 2, 2, 100.0, 50.0, 1.0, 1.0, 0.0);
  const Mesh box = demdl::mesh::solidMesh(surface, 2, 2, -1.0f);
  REQUIRE(box.triangles.size() == 12);
  const BlenderJob job{dir.path / "box.ply", dir.path / "box.stl", dir.path / "box.blend", "box",
                       100.0,               50.0,                10.0,                  10.0, 1.0};
  demdl::mesh::writeBinaryPly(surface, job.ply, 100.0, 50.0);
  const auto check = [&blender, &job](const Mesh& mesh, double scale) {
    demdl::mesh::writeBinaryStl(mesh, job.stl, "dem2mesh test box");
    BlenderJob scaled = job;
    scaled.scale = scale;
    return runBlender(*blender, scaled);
  };

  {
    const BlenderResult result = check(box, 1.0);
    INFO("closed, output:\n" << result.output);
    CHECK(result.exitCode == 0);
    CHECK_THAT(result.output, ContainsSubstring("\nstl check: 12 triangles, 0 non-manifold edges, 0 inverted faces\n"));
    CHECK(readFile(job.blend).substr(0, 4) == "\x28\xb5\x2f\xfd");  // saved compressed, with Zstandard
    // Where the PLY and the STL put it, so that the STL imported beside it lies on it; its origin in the middle.
    CHECK_THAT(result.output,
               ContainsSubstring(", in place at x 0 to 100, y 0 to 50, z 10 to 10, origin at the centre of its base\n"));
    // At full size a unit is a metre.
    CHECK_THAT(result.output, ContainsSubstring("\nblender: units: METRIC, scale_length 1, length_unit METERS\n"));
  }
  {
    // Without a triangle of the base, whose three edges are left with one face each, and with a triangle of the top
    // wound the other way.
    Mesh broken = box;
    broken.triangles.pop_back();
    std::swap(broken.triangles[0].b, broken.triangles[0].c);
    const BlenderResult result = check(broken, 0.5);
    INFO("open and twisted, output:\n" << result.output);
    CHECK(result.exitCode == 1);
    CHECK_THAT(result.output, ContainsSubstring("\nstl check: 11 triangles, 3 non-manifold edges, 1 inverted faces\n"));
    // Half size: a unit is 2 m of terrain.
    CHECK_THAT(result.output, ContainsSubstring("\nblender: units: METRIC, scale_length 2, length_unit METERS\n"));
  }
  {
    // Every triangle wound the other way is still closed and consistent, so only the turn to face outward catches it.
    Mesh insideOut = box;
    for (demdl::mesh::Triangle& t : insideOut.triangles) std::swap(t.b, t.c);
    const BlenderResult result = check(insideOut, 0.05);
    INFO("inside out, output:\n" << result.output);
    CHECK(result.exitCode == 1);
    CHECK_THAT(result.output,
               ContainsSubstring("\nstl check: 12 triangles, 0 non-manifold edges, 12 inverted faces\n"));
    // A print: a unit shows as the millimetre the slicer prints it as.
    CHECK_THAT(result.output,
               ContainsSubstring("\nblender: units: METRIC, scale_length 0.001, length_unit MILLIMETERS\n"));
  }
  {
    // A triangle twice over, which Blender's reader drops without failing; only the count in the file's header shows
    // it. The rest of the check passes on what was read.
    Mesh repeated = box;
    repeated.triangles.push_back(repeated.triangles[0]);
    const BlenderResult result = check(repeated, 0.1);
    INFO("a triangle repeated, output:\n" << result.output);
    CHECK(result.exitCode == 1);
    CHECK_THAT(result.output, ContainsSubstring("\nstl check: file declares 13 triangles, Blender imported 12\n"));
    CHECK_THAT(result.output, ContainsSubstring("\nstl check: 12 triangles, 0 non-manifold edges, 0 inverted faces\n"));
    // A tenth is not yet print-sized: a unit is 10 m of terrain.
    CHECK_THAT(result.output, ContainsSubstring("\nblender: units: METRIC, scale_length 10, length_unit METERS\n"));
  }
  // Each run after the first saved over the scene, which Blender would otherwise have kept as box.blend1.
  std::vector<std::string> names;
  for (const auto& entry : std::filesystem::directory_iterator(dir.path)) names.push_back(entry.path().filename().string());
  std::sort(names.begin(), names.end());
  CHECK(names == std::vector<std::string>{"box.blend", "box.ply", "box.stl"});
}
