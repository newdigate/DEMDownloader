#include "mesh/BlenderRunner.h"

#include <fcntl.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <system_error>
#include <vector>

#include "mesh/BlenderScenePy.h"

extern char** environ;

namespace demdl::mesh {

namespace {

namespace fs = std::filesystem;

// '<text>', so that an empty path shows as '' rather than as nothing.
std::string quoted(const std::string& text) { return "'" + text + "'"; }

std::string reason(int error) { return std::generic_category().message(error); }

// The 17 significant digits that read back as the same double.
std::string exactNumber(double value) {
  char text[40];
  std::snprintf(text, sizeof text, "%.17g", value);
  return text;
}

// path from the working directory when it is relative. As it is when the working directory cannot be had, which
// leaves Blender no worse off.
std::string absolutePath(const fs::path& path) {
  std::error_code ec;
  const fs::path full = fs::absolute(path, ec);
  return ec ? path.string() : full.string();
}

// An end of the pipe, closed when this goes unless closed before. Nothing is written through it here, so a failure
// to close it loses nothing.
class Descriptor {
 public:
  Descriptor() = default;
  ~Descriptor() { close(); }
  Descriptor(const Descriptor&) = delete;
  Descriptor& operator=(const Descriptor&) = delete;

  void reset(int fd) {
    close();
    fd_ = fd;
  }
  [[nodiscard]] int get() const { return fd_; }
  void close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }

 private:
  int fd_ = -1;
};

// A file for Blender to read, which runBlender makes in the temporary directory and removes when this goes: on every
// way out of runBlender.
class TemporaryFile {
 public:
  TemporaryFile() = default;
  ~TemporaryFile() {
    if (!path_.empty()) ::unlink(path_.c_str());
  }
  TemporaryFile(const TemporaryFile&) = delete;
  TemporaryFile& operator=(const TemporaryFile&) = delete;

  // Writes text to a new file in the temporary directory, called prefix, six random characters and suffix:
  // dem2mesh-q1W2e3.py for "dem2mesh-" and ".py". Empty when done, otherwise what went wrong, with `what` naming the
  // file for the message.
  std::string write(const std::string& prefix, const std::string& suffix, std::string_view text,
                    const std::string& what) {
    std::error_code ec;
    const fs::path dir = fs::temp_directory_path(ec);
    if (ec) return "cannot find a temporary directory for " + what + ": " + ec.message();
    // mkstemps puts six random characters for the Xs and creates the file for this user alone, failing rather than
    // opening one that is there already, so no other process can put its own file in its place.
    std::string name = (dir / (prefix + "XXXXXX" + suffix)).string();
    const int fd = ::mkstemps(name.data(), static_cast<int>(suffix.size()));
    if (fd < 0) {
      const int error = errno;
      return "cannot create " + what + " in " + dir.string() + ": " + reason(error);
    }
    path_ = name;
    int error = 0;
    for (std::string_view rest = text; !rest.empty() && error == 0;) {
      const ssize_t written = ::write(fd, rest.data(), rest.size());
      if (written >= 0) {
        rest.remove_prefix(static_cast<std::size_t>(written));
      } else if (errno != EINTR) {
        error = errno;
      }
    }
    // The close can be the first to report that the text did not reach the file, so it is checked too.
    if (::close(fd) != 0 && error == 0) error = errno;
    if (error != 0) return "cannot write " + what + " " + path_ + ": " + reason(error);
    return {};
  }

  [[nodiscard]] const std::string& path() const { return path_; }

 private:
  std::string path_;
};

// Starts blender with args, its stdin from /dev/null and its stdout and stderr both into out. 0, or the error.
int spawn(const fs::path& blender, const std::vector<std::string>& args, int out, pid_t& pid) {
  std::vector<char*> argv{const_cast<char*>(blender.c_str())};
  for (const std::string& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);

  posix_spawn_file_actions_t actions;
  if (const int error = posix_spawn_file_actions_init(&actions); error != 0) return error;
  posix_spawnattr_t attributes;
  if (const int error = posix_spawnattr_init(&attributes); error != 0) {
    posix_spawn_file_actions_destroy(&actions);
    return error;
  }
  int error = posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  if (error == 0) error = posix_spawn_file_actions_adddup2(&actions, out, STDOUT_FILENO);
  if (error == 0) error = posix_spawn_file_actions_adddup2(&actions, out, STDERR_FILENO);
#ifdef POSIX_SPAWN_CLOEXEC_DEFAULT
  // macOS: Blender inherits those three descriptors and no others. Elsewhere the pipe's own ends are close-on-exec.
  if (error == 0) error = posix_spawnattr_setflags(&attributes, POSIX_SPAWN_CLOEXEC_DEFAULT);
#endif
  // posix_spawnp, so that a name without a "/", from --blender or BLENDER, is looked for on PATH as a shell would.
  if (error == 0) error = posix_spawnp(&pid, blender.c_str(), &actions, &attributes, argv.data(), environ);
  posix_spawnattr_destroy(&attributes);
  posix_spawn_file_actions_destroy(&actions);
  return error;
}

// What arrives through fd until every writer has closed it, which the child does by ending. A read that fails ends
// the reading, with a line saying so.
std::string readAll(int fd) {
  std::string text;
  std::vector<char> buffer(std::size_t{1} << 16);
  for (;;) {
    const ssize_t got = ::read(fd, buffer.data(), buffer.size());
    if (got > 0) {
      text.append(buffer.data(), static_cast<std::size_t>(got));
    } else if (got == 0) {
      return text;
    } else if (const int error = errno; error != EINTR) {
      return text + "\ncannot read Blender's output: " + reason(error) + "\n";
    }
  }
}

// text with a newline at its end, so that a line added after it starts a line of its own.
void endLine(std::string& text) {
  if (!text.empty() && text.back() != '\n') text += '\n';
}

}  // namespace

std::optional<fs::path> namedBlender(const std::optional<fs::path>& flag) {
  if (flag) return flag;
  if (const char* env = std::getenv("BLENDER"); env != nullptr && *env != '\0') return fs::path(env);
  return std::nullopt;
}

std::optional<fs::path> findBlender(const std::optional<fs::path>& flag) {
  if (std::optional<fs::path> named = namedBlender(flag)) return named;
  std::error_code ignored;
  if (fs::exists(kBlenderAppExecutable, ignored)) return fs::path(kBlenderAppExecutable);
  const char* path = std::getenv("PATH");
  return findOnPath("blender", path != nullptr ? path : "");
}

std::optional<fs::path> findOnPath(std::string_view name, std::string_view searchPath) {
  for (;;) {
    const std::size_t colon = searchPath.find(':');
    if (const std::string_view dir = searchPath.substr(0, colon); !dir.empty()) {
      const fs::path candidate = fs::path(dir) / name;
      std::error_code ignored;
      if (fs::is_regular_file(candidate, ignored) && ::access(candidate.c_str(), X_OK) == 0) return candidate;
    }
    if (colon == std::string_view::npos) return std::nullopt;
    searchPath.remove_prefix(colon + 1);
  }
}

std::optional<std::string> blenderStartProblem(const fs::path& blender) {
  const std::string text = blender.string();
  if (text.find('/') == std::string::npos) {
    const char* path = std::getenv("PATH");
    if (findOnPath(text, path != nullptr ? path : "")) return std::nullopt;
    return "not found on PATH";
  }
  std::error_code ec;
  const fs::file_status status = fs::status(blender, ec);  // through any links, as starting it would go
  if (status.type() == fs::file_type::not_found) return "does not exist";
  if (ec) return "cannot be looked at (" + ec.message() + ")";
  if (fs::is_directory(status)) {
    // The path as typed, without the slash that a shell's completion leaves after a directory.
    std::string bundle = text;
    while (bundle.size() > 1 && bundle.back() == '/') bundle.pop_back();
    if (bundle.ends_with(".app")) return "is the app bundle; its executable is " + bundle + "/Contents/MacOS/Blender";
  }
  if (!fs::is_regular_file(status)) return "is not a regular file";
  if (::access(blender.c_str(), X_OK) != 0) return "is not executable";
  return std::nullopt;
}

BlenderResult runBlender(const fs::path& blender, const BlenderJob& job) {
  const std::string cannotStart = "cannot start " + quoted(blender.string()) + ": ";
  if (job.stls.empty()) return {127, cannotStart + "the job names no STL to check\n"};
  TemporaryFile script;
  if (const std::string problem = script.write("dem2mesh-", ".py", kBlenderScenePy, "the scene script");
      !problem.empty()) {
    return {127, cannotStart + problem + "\n"};
  }
  // One STL goes to the script as it is. Several go in a file beside it, one absolute path to a line, so that the
  // command line is as long for a puzzle of ten thousand blocks as for one.
  TemporaryFile list;
  if (job.stls.size() > 1) {
    std::string paths;
    for (const fs::path& stl : job.stls) paths += absolutePath(stl) + "\n";
    if (const std::string problem = list.write("dem2mesh-stls-", ".txt", paths, "the list of STL files");
        !problem.empty()) {
      return {127, cannotStart + problem + "\n"};
    }
  }

  // Close-on-exec on both ends: the child is to have the write end as its stdout and stderr only, or the pipe would
  // never see its end while a stray copy stayed open.
  int ends[2];
  if (::pipe(ends) != 0) {
    const int error = errno;
    return {127, cannotStart + "no pipe for its output: " + reason(error) + "\n"};
  }
  Descriptor readEnd;
  Descriptor writeEnd;
  readEnd.reset(ends[0]);
  writeEnd.reset(ends[1]);
  for (const int fd : ends) {
    if (::fcntl(fd, F_SETFD, FD_CLOEXEC) != 0) {
      const int error = errno;
      return {127, cannotStart + "no pipe for its output: " + reason(error) + "\n"};
    }
  }

  const std::string solid = job.stls.size() == 1 ? "--stl=" + absolutePath(job.stls.front())
                                                 : "--stl-list=" + absolutePath(list.path());
  const std::vector<std::string> args{"--background",
                                      "--factory-startup",
                                      "--python",
                                      script.path(),
                                      "--",
                                      "--ply=" + absolutePath(job.ply),
                                      solid,
                                      "--blend=" + absolutePath(job.blend),
                                      "--name=" + job.name,
                                      "--width=" + exactNumber(job.widthUnits),
                                      "--height=" + exactNumber(job.heightUnits),
                                      "--zmin=" + exactNumber(job.minZ),
                                      "--zmax=" + exactNumber(job.maxZ),
                                      "--scale=" + exactNumber(job.scale)};
  pid_t pid = -1;
  if (const int error = spawn(blender, args, writeEnd.get(), pid); error != 0) {
    return {127, cannotStart + reason(error) + "\n"};
  }
  // Only the child may hold the write end now, so that the pipe ends when the child does.
  writeEnd.close();
  BlenderResult result{0, readAll(readEnd.get())};
  readEnd.close();  // a child still writing after a failed read gets SIGPIPE rather than blocking the wait below

  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (const int error = errno; error != EINTR) {
      endLine(result.output);
      result.output += "cannot learn how " + quoted(blender.string()) + " ended: " + reason(error) + "\n";
      result.exitCode = 1;
      return result;
    }
  }
  if (WIFSIGNALED(status)) {
    endLine(result.output);
    result.output += quoted(blender.string()) + " was ended by signal " + std::to_string(WTERMSIG(status)) + "\n";
    result.exitCode = 128 + WTERMSIG(status);
  } else {
    result.exitCode = WEXITSTATUS(status);
  }
  return result;
}

}  // namespace demdl::mesh
