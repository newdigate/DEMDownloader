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

// The stand-in's executable and what it leaves beside it: the arguments it was given, one per line, and a copy of the
// file that followed --python, taken while it ran.
struct FakeBlender {
  std::filesystem::path executable;
  std::filesystem::path arguments;
  std::filesystem::path scriptCopy;

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
// when that file cannot be copied; prints "fake blender: to stdout" on stdout, then "fake blender: to stderr" on stderr;
// and exits with $FAKE_EXIT, 0 when that is unset. With FAKE_SIGNAL set it kills itself with that signal instead. With
// FAKE_STL_CHECK set it acts out a Blender that did its work: it writes a file beginning "BLENDER" at its --blend=
// argument, unless FAKE_NO_BLEND is set too, and prints the variable's value as a line. It names cp by its path, so
// that it runs whatever PATH is.
inline FakeBlender writeFakeBlender(const std::filesystem::path& dir) {
  const FakeBlender fake{dir / "blender", dir / "arguments.txt", dir / "script-copy.py"};
  // The paths go into the script between single quotes.
  REQUIRE(dir.string().find('\'') == std::string::npos);
  const std::string arguments = "'" + fake.arguments.string() + "'";
  const std::string scriptCopy = "'" + fake.scriptCopy.string() + "'";
  writeFile(fake.executable,
            "#!/bin/sh\n"
            ": > " + arguments + "\n"
            "previous=''\n"
            "blend=''\n"
            "for arg in \"$@\"; do\n"
            "  printf '%s\\n' \"$arg\" >> " + arguments + "\n"
            "  if [ \"$previous\" = --python ]; then /bin/cp \"$arg\" " + scriptCopy + " || exit 98; fi\n"
            "  case \"$arg\" in --blend=*) blend=${arg#--blend=} ;; esac\n"
            "  previous=$arg\n"
            "done\n"
            "echo 'fake blender: to stdout'\n"
            "echo 'fake blender: to stderr' >&2\n"
            "if [ -n \"$FAKE_SIGNAL\" ]; then kill -\"$FAKE_SIGNAL\" $$; fi\n"
            "if [ -n \"$FAKE_STL_CHECK\" ]; then\n"
            "  if [ -z \"$FAKE_NO_BLEND\" ]; then printf 'BLENDER-fake' > \"$blend\" || exit 97; fi\n"
            "  printf '%s\\n' \"$FAKE_STL_CHECK\"\n"
            "fi\n"
            "exit \"${FAKE_EXIT:-0}\"\n");
  using std::filesystem::perms;
  std::filesystem::permissions(fake.executable, perms::owner_all | perms::group_read | perms::group_exec |
                                                    perms::others_read | perms::others_exec);
  return fake;
}
