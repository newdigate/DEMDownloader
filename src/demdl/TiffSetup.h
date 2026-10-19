#pragma once

#include <string>

namespace demdl {

// libtiff's process-wide setup, shared by everything in this repository that opens a TIFF. The first call registers
// the GDAL_NODATA tag beside libgeotiff's GeoTIFF tags on every handle opened after it (so it must come before a file
// that may carry the tag is opened, for reading or writing), installs an error handler that collects libtiff's errors
// for takeTiffError() instead of printing them, and silences libtiff's warnings. Later calls do nothing; any thread
// may make the first.
void ensureTagsRegistered();

// The libtiff errors raised on this thread since the last call, joined by "; ", which are then forgotten.
[[nodiscard]] std::string takeTiffError();

}  // namespace demdl
