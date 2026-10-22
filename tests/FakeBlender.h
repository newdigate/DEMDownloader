#pragma once

// A stand-in for Blender, for the tests of BlenderRunner and of how dem2mesh deals with Blender, and a guard that sets
// an environment variable for one test. The stand-in is a shell script, so those tests need no Blender and behave the
// same on a machine that has one.

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "SpawnedProcess.h"  // writeFile

// Sets an environment variable, or unsets it when value is nullopt, until the guard goes, which puts back what was
// there before. Child processes started meanwhile see the change, since they are given this process's environment.
class ScopedEnv {
 public:
  ScopedEnv(std::string name, const std::optional<std::string>& value) : name_(std::move(name)) {
    if (const char* old = std::getenv(name_.c_str())) old_ = old;
    if (value) {
      REQUIRE(setenv(name_.c_str(), value->c_str(), 1) == 0);
    } else {
      REQUIRE(unsetenv(name_.c_str()) == 0);
    }
  }
  ~ScopedEnv() {
    if (old_) {
      setenv(name_.c_str(), old_->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }
  ScopedEnv(const ScopedEnv&) = delete;
  ScopedEnv& operator=(const ScopedEnv&) = delete;

 private:
  std::string name_;
  std::optional<std::string> old_;
};

// The stand-in's executable and what it leaves beside it: the arguments it was given, one per line, a copy of the
// file that followed --python, taken while it ran, and a copy of the list that --stl-list= named, if it was given one.
struct FakeBlender {
  std::filesystem::path executable;
  std::filesystem::path arguments;
  std::filesystem::path scriptCopy;
  std::filesystem::path listCopy;

  // The arguments of its last run, in order.
  [[nodiscard]] std::vector<std::string> recordedArguments() const {
    std::ifstream in(arguments);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    return lines;
  }
};

// The number in an argument "<option>=<number>", read back as a double. Fails the test when it is not one.
inline double numberIn(const std::string& argument, const std::string& option) {
  INFO(argument);
  REQUIRE(argument.starts_with(option + "="));
  const std::string text = argument.substr(option.size() + 1);
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  REQUIRE(end != text.c_str());
  REQUIRE(*end == '\0');
  return value;
}

// Writes the stand-in as dir/blender, executable. It records its arguments and copies its script as above, exiting 98
// when that file cannot be copied, and copies the list of an --stl-list= argument the same way, exiting 96 when it
// cannot; prints "fake blender: to stdout" on stdout, then "fake blender: to stderr" on stderr; and exits with
// $FAKE_EXIT, 0 when that is unset. With FAKE_SIGNAL set it kills itself with that signal instead. With FAKE_STL_CHECK
// set it acts out a Blender that did its work: it writes a file beginning "BLENDER" at its --blend= argument, unless
// FAKE_NO_BLEND is set too, and prints one line of check. That is the variable's value for an --stl= argument. For an
// --stl-list= it is the totals of the files that the list names, as Blender's script gives them, and the variable's
// value is not used: "stl check: N files, T triangles, 0 non-manifold edges, 0 inverted faces", T being the sum of the
// triangle counts that the files' headers declare (the four bytes at 80), so that a test can tell which files the list
// held. A file in it that cannot be read ends the stand-in with exit 95. It names cp and od by their paths, so that it
// runs whatever PATH is.
inline FakeBlender writeFakeBlender(const std::filesystem::path& dir) {
  const FakeBlender fake{dir / "blender", dir / "arguments.txt", dir / "script-copy.py", dir / "list-copy.txt"};
  // The paths go into the script between single quotes.
  REQUIRE(dir.string().find('\'') == std::string::npos);
  const std::string arguments = "'" + fake.arguments.string() + "'";
  const std::string scriptCopy = "'" + fake.scriptCopy.string() + "'";
  const std::string listCopy = "'" + fake.listCopy.string() + "'";
  writeFile(fake.executable,
            "#!/bin/sh\n"
            ": > " + arguments + "\n"
            "previous=''\n"
            "blend=''\n"
            "list=''\n"
            "for arg in \"$@\"; do\n"
            "  printf '%s\\n' \"$arg\" >> " + arguments + "\n"
            "  if [ \"$previous\" = --python ]; then /bin/cp \"$arg\" " + scriptCopy + " || exit 98; fi\n"
            "  case \"$arg\" in\n"
            "    --blend=*) blend=${arg#--blend=} ;;\n"
            "    --stl-list=*) list=${arg#--stl-list=} ;;\n"
            "  esac\n"
            "  previous=$arg\n"
            "done\n"
            "if [ -n \"$list\" ]; then /bin/cp \"$list\" " + listCopy + " || exit 96; fi\n"
            "echo 'fake blender: to stdout'\n"
            "echo 'fake blender: to stderr' >&2\n"
            "if [ -n \"$FAKE_SIGNAL\" ]; then kill -\"$FAKE_SIGNAL\" $$; fi\n"
            "if [ -n \"$FAKE_STL_CHECK\" ]; then\n"
            "  if [ -z \"$FAKE_NO_BLEND\" ]; then printf 'BLENDER-fake' > \"$blend\" || exit 97; fi\n"
            "  if [ -n \"$list\" ]; then\n"
            "    files=0\n"
            "    triangles=0\n"
            "    while IFS= read -r stl; do\n"
            "      bytes=$(/usr/bin/od -An -tu1 -j80 -N4 \"$stl\") || exit 95\n"
            "      set -- $bytes\n"
            "      triangles=$((triangles + $1 + 256 * $2 + 65536 * $3 + 16777216 * $4))\n"
            "      files=$((files + 1))\n"
            "    done < \"$list\"\n"
            "    printf 'stl check: %s files, %s triangles, 0 non-manifold edges, 0 inverted faces\\n' \"$files\" "
            "\"$triangles\"\n"
            "  else\n"
            "    printf '%s\\n' \"$FAKE_STL_CHECK\"\n"
            "  fi\n"
            "fi\n"
            "exit \"${FAKE_EXIT:-0}\"\n");
  using std::filesystem::perms;
  std::filesystem::permissions(fake.executable, perms::owner_all | perms::group_read | perms::group_exec |
                                                    perms::others_read | perms::others_exec);
  return fake;
}
