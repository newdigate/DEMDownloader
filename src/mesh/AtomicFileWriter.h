#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string_view>
#include <vector>

namespace demdl::mesh {

// The files are defined as IEEE 754 single precision; a host with another float format would write garbage.
static_assert(std::numeric_limits<float>::is_iec559 && sizeof(float) == sizeof(std::uint32_t));

// A binary file for the STL and PLY writers: everything goes to <path>.tmp, which is renamed over <path> only once
// commit() has seen every byte written and the file closed without error, without the fsync DemRaster::save makes for
// its checkpoint. So a reader never meets a half-written file, an existing file survives a write that fails, and a
// failure leaves no .tmp behind: the destructor removes it unless commit() finished. A power cut just after the rename
// can still leave a short file, which a run made again replaces.
//
// A number is written least significant byte first whatever the host's byte order, and a float as the bytes of its
// IEEE 754 single-precision bit pattern, so a file is the same bytes on every machine. Writes gather in a buffer of
// kBufferBytes that goes to the file when it fills: a 25-million-vertex mesh is never held a second time as a file
// image, and the file sees one large write in place of a call per float.
//
// Every failure is a std::runtime_error that names the file and, when the library kept one, the operating system's
// reason.
class AtomicFileWriter {
 public:
  static constexpr std::size_t kBufferBytes = std::size_t{1} << 20;

  // Opens <path>.tmp for writing, replacing one left by an earlier run that died. Throws when it cannot be created: a
  // missing directory, no permission.
  explicit AtomicFileWriter(const std::filesystem::path& path);
  ~AtomicFileWriter();
  AtomicFileWriter(const AtomicFileWriter&) = delete;
  AtomicFileWriter& operator=(const AtomicFileWriter&) = delete;

  void u8(std::uint8_t value) {
    makeRoom(1);
    buffer_[used_++] = value;
  }
  void u16(std::uint16_t value) {
    makeRoom(2);
    std::uint8_t* at = buffer_.data() + used_;
    at[0] = static_cast<std::uint8_t>(value);
    at[1] = static_cast<std::uint8_t>(value >> 8);
    used_ += 2;
  }
  void u32(std::uint32_t value) {
    makeRoom(4);
    std::uint8_t* at = buffer_.data() + used_;
    at[0] = static_cast<std::uint8_t>(value);
    at[1] = static_cast<std::uint8_t>(value >> 8);
    at[2] = static_cast<std::uint8_t>(value >> 16);
    at[3] = static_cast<std::uint8_t>(value >> 24);
    used_ += 4;
  }
  void f32(float value) { u32(std::bit_cast<std::uint32_t>(value)); }
  void text(std::string_view chars);
  void zeros(std::size_t count);

  // Writes what is gathered, closes the file and renames it over the destination. Throws if any of that fails, and
  // then the destructor removes the .tmp. Nothing may be written after it returns.
  void commit();

 private:
  void makeRoom(std::size_t bytes) {
    if (kBufferBytes - used_ < bytes) flush();
  }
  void flush();

  std::filesystem::path path_;
  std::filesystem::path tmp_;
  std::ofstream out_;
  std::vector<std::uint8_t> buffer_;
  std::size_t used_ = 0;
  bool committed_ = false;
};

}  // namespace demdl::mesh
