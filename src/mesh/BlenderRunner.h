#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace demdl::mesh {

// Where the Blender app keeps its executable on macOS: the third place findBlender looks.
inline constexpr char kBlenderAppExecutable[] = "/Applications/Blender.app/Contents/MacOS/Blender";

// The Blender the user named (spec 7.1): flag, which is --blender, when given, as it is; else $BLENDER when it is set
// and not empty, also as it is; else nullopt. A Blender named is used with no fallback, so that a bad one fails with
// its name rather than some other Blender running in its place.
[[nodiscard]] std::optional<std::filesystem::path> namedBlender(const std::optional<std::filesystem::path>& flag);

// The Blender to run (spec 7.1), in this order:
// - the one the user named, as namedBlender gives it. An empty --blender counts as named, though the parser refuses
//   one, so that it can never fall through to another Blender.
// - kBlenderAppExecutable when it exists.
// - the first blender on PATH, as findOnPath finds it.
// nullopt when none of them gives one. Whether a named one can be started is for blenderStartProblem to say before the
// run reads anything; whether the one run does start is for runBlender to find out, and to report.
[[nodiscard]] std::optional<std::filesystem::path> findBlender(const std::optional<std::filesystem::path>& flag);

// Why the Blender the user named cannot be started, as far as can be told without starting it; nullopt when it can be.
// The text completes "--blender <path>: " or "BLENDER=<path>: ":
// - a name without a "/" is looked for on PATH with findOnPath: "not found on PATH";
// - any other path, from the working directory when it is relative, must exist ("does not exist"), be a regular file
//   once links are followed ("is not a regular file", or for a directory whose name ends in ".app" "is the app bundle;
//   its executable is <path>/Contents/MacOS/Blender") and be one this process may execute ("is not executable"). A
//   path that cannot be looked at, behind a directory this user may not search for instance, "cannot be looked at
//   (<reason>)".
[[nodiscard]] std::optional<std::string> blenderStartProblem(const std::filesystem::path& blender);

// The first directory of searchPath, a list separated by ':' as PATH is, that holds `name` as a regular file this
// process may execute (a link to one counts), joined to the name; nullopt when none does. An empty entry is skipped,
// where a shell would take it for the working directory, so that a stray program beside the user is never run unasked.
[[nodiscard]] std::optional<std::filesystem::path> findOnPath(std::string_view name, std::string_view searchPath);

// What Blender is asked to make (spec 7.2): a scene of the terrain surface in ply, saved at blend, the object called
// name; framed on the window, widthUnits by heightUnits, and the surface's heights, minZ to maxZ, all in model units;
// its units set from scale, which is --scale, model units per metre. Then the solid in stl is checked.
struct BlenderJob {
  std::filesystem::path ply, stl, blend;
  std::string name;
  double widthUnits, heightUnits, minZ, maxZ, scale;
};

// How a Blender run ended: its exit code, and everything it wrote to stdout and stderr, as one text in the order it
// was written.
struct BlenderResult {
  int exitCode;
  std::string output;
};

// Runs blender on the scene script embedded at build time (blender_scene.py), with the job after "--":
//   <blender> --background --factory-startup --python <script> -- --ply=<ply> --stl=<stl> --blend=<blend>
//       --name=<name> --width=<widthUnits> --height=<heightUnits> --zmin=<minZ> --zmax=<maxZ> --scale=<scale>
// - The script is written to a new file, dem2mesh-XXXXXX.py in the temporary directory, which is removed before this
//   returns, however it returns.
// - Each value is joined to its option by "=", so that a negative number or a name that starts with "-" is never
//   taken for an option. The paths are made absolute, so that Blender finds them whatever it takes a relative path to
//   be relative to. The numbers carry every digit a double needs, so the script reads the very same numbers.
// - blender is looked up as a shell would look it up: a name without a "/" in it is searched for on PATH.
// - Blender's stdin is /dev/null, and its stdout and stderr share one pipe, read here to the end, so it shows nothing
//   on the terminal.
// The exit code is Blender's own; 128 plus the signal's number when a signal ended it, with a line saying so added to
// the output; 127 when it could not be started at all (no such file, not executable, or no script file or pipe to be
// had), with the reason, naming the executable, as the output; 1, with a line saying so, when how it ended cannot be
// learnt. Throws nothing but std::bad_alloc.
[[nodiscard]] BlenderResult runBlender(const std::filesystem::path& blender, const BlenderJob& job);

}  // namespace demdl::mesh
