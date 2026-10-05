#pragma once

#include <iosfwd>

#include "demdl/Config.h"

namespace demdl::cli {

struct [[nodiscard]] ParseOutcome {
  bool exitNow = false;  // true after --help, --version or a usage error
  int exitCode = 0;      // 0 for help and version, 1 for a usage error
};

// Fills `config` from argv with CLI11. Help, version and error text go to the given streams.
// Does not call Config::validate(); the caller does. After a usage error `config` may be partially
// filled and must not be used.
[[nodiscard]] ParseOutcome parseArgs(int argc, const char* const* argv, Config& config, std::ostream& out,
                                     std::ostream& err);

}  // namespace demdl::cli
