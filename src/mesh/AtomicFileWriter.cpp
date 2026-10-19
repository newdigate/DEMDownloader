#include "mesh/AtomicFileWriter.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

namespace demdl::mesh {

namespace {

// Throws what, then " (<reason>)" when osError, an errno value, is not zero. The standard promises no errno from
// iostreams; libc++ leaves the failing call's, which the tests check, and where a library does not, the message has no
// reason rather than a wrong one.
[[noreturn]] void fail(const std::string& what, int osError) {
  std::string message = what;
  if (osError != 0) message += " (" + std::generic_category().message(osError) + ")";
  throw std::runtime_error(message);
}

}  // namespace

AtomicFileWriter::AtomicFileWriter(const std::filesystem::path& path)
    : path_(path), tmp_(path.string() + ".tmp"), buffer_(kBufferBytes) {
  // errno is cleared just before each call that can fail in the operating system and read just after a failure, as in
  // DemRaster::save, so that a missing directory or a full disk is named as the cause.
  errno = 0;
  out_.open(tmp_, std::ios::binary | std::ios::trunc);
  const int openError = errno;
  if (!out_) fail("cannot create " + tmp_.string(), openError);
}

AtomicFileWriter::~AtomicFileWriter() {
  if (committed_) return;
  out_.close();  // before the remove, so that the file is not open while it goes; a failure to close is moot now
  std::error_code ignored;
  std::filesystem::remove(tmp_, ignored);
}

void AtomicFileWriter::text(std::string_view chars) {
  while (!chars.empty()) {
    makeRoom(1);
    const std::size_t n = std::min(chars.size(), kBufferBytes - used_);
    std::memcpy(buffer_.data() + used_, chars.data(), n);
    used_ += n;
    chars.remove_prefix(n);
  }
}

void AtomicFileWriter::zeros(std::size_t count) {
  while (count > 0) {
    makeRoom(1);
    const std::size_t n = std::min(count, kBufferBytes - used_);
    std::memset(buffer_.data() + used_, 0, n);
    used_ += n;
    count -= n;
  }
}

void AtomicFileWriter::flush() {
  errno = 0;
  out_.write(reinterpret_cast<const char*>(buffer_.data()), static_cast<std::streamsize>(used_));
  const int writeError = errno;
  if (!out_) fail("write failed on " + tmp_.string(), writeError);
  used_ = 0;
}

void AtomicFileWriter::commit() {
  flush();
  // The stream keeps a buffer of its own, so the last bytes, and any error in writing them, are still to come out of
  // it. Closing here writes them, and fails the stream if that or the close fails; doing so before the rename is what
  // keeps a failed write from ever replacing the file.
  errno = 0;
  out_.close();
  const int closeError = errno;
  if (out_.fail()) fail("cannot finish writing " + tmp_.string(), closeError);

  std::error_code ec;
  std::filesystem::rename(tmp_, path_, ec);
  if (ec) {
    throw std::runtime_error("cannot rename " + tmp_.string() + " to " + path_.string() + ": " + ec.message());
  }
  committed_ = true;
}

}  // namespace demdl::mesh
