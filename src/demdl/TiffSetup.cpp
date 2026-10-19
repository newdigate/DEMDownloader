#include "demdl/TiffSetup.h"

#include <tiffio.h>
#include <xtiffio.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>

namespace demdl {

namespace {

// The libtiff errors raised on this thread since takeTiffError() last ran, joined by "; ".
thread_local std::string lastTiffError;

// Installed in place of libtiff's default handler, which prints to stderr, so that the text reaches
// the user in an exception message instead of interrupting the progress output. errno is preserved
// for the caller, which reports the operating system's reason for a failed write.
void captureTiffError(const char* module, const char* fmt, va_list ap) {
  const int savedErrno = errno;
  char text[1024];
  std::vsnprintf(text, sizeof text, fmt, ap);
  try {
    if (!lastTiffError.empty()) lastTiffError += "; ";
    if (module != nullptr) lastTiffError += std::string(module) + ": ";
    lastTiffError += text;
  } catch (...) {
    // Out of memory: the text is lost, but the caller still reports the failure itself.
  }
  errno = savedErrno;
}

// libtiff does not know the GDAL_NODATA tag: TIFFSetField refuses it and a reader sees an anonymous
// field. This tag extender registers it on every TIFF handle opened after ensureTagsRegistered(),
// then calls the extender installed before it, libgeotiff's, which registers the GeoTIFF tags.
TIFFExtendProc g_parentExtender = nullptr;

void registerGdalTags(TIFF* tif) {
  static const TIFFFieldInfo fields[] = {
      {TIFFTAG_GDAL_NODATA, -1, -1, TIFF_ASCII, FIELD_CUSTOM, 1, 0, const_cast<char*>("GDALNoDataValue")},
  };
  TIFFMergeFieldInfo(tif, fields, 1);
  if (g_parentExtender != nullptr) g_parentExtender(tif);
}

}  // namespace

std::string takeTiffError() {
  std::string text;
  text.swap(lastTiffError);
  return text;
}

void ensureTagsRegistered() {
  static std::once_flag once;
  std::call_once(once, [] {
    // libgeotiff's first-time setup is not thread-safe, so it is forced to run here, once. The
    // extender order is therefore fixed: ours runs first, then libgeotiff's.
    XTIFFInitialize();
    g_parentExtender = TIFFSetTagExtender(registerGdalTags);
    TIFFSetErrorHandler(captureTiffError);
    // Our own files raise no warnings; another program's file would otherwise print them while it is read.
    TIFFSetWarningHandler(nullptr);
  });
}

}  // namespace demdl
