#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <signal.h>
#include <sys/resource.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "ExampleConfig.h"
#include "TempDir.h"
#include "demdl/Config.h"
#include "demdl/DemRaster.h"

using Catch::Matchers::ContainsSubstring;
using demdl::BoundingBox;
using demdl::Config;
using demdl::DemRaster;
using demdl::RasterMetadata;

namespace {
const BoundingBox kBox{-34.0, -34.3, 18.5, 18.1};
}  // namespace

TEST_CASE("a new raster is entirely NaN", "[demraster]") {
  const auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  CHECK(r.width() == 4);
  CHECK(r.height() == 3);
  CHECK(r.pixelCount() == 12);
  CHECK(r.countNaN() == 12);
  CHECK(std::isnan(r.at(3, 2)));
  CHECK(r.box().approxEquals(kBox, 0.0));
  CHECK(r.metadata().layer == "LayerA");
  CHECK(r.metadata().wmsUrl == "https://example.test/wms");
}

TEST_CASE("set and at address pixels by column and row", "[demraster]") {
  auto r = DemRaster::create(4, 3, kBox, {});
  r.set(1, 2, 12.5f);
  CHECK(r.at(1, 2) == 12.5f);
  CHECK(std::isnan(r.at(2, 1)));
  CHECK(r.countNaN() == 11);
}

TEST_CASE("todoInRow and rowsWithWork describe the remaining NaN pixels", "[demraster]") {
  auto r = DemRaster::create(4, 3, kBox, {});
  for (int x = 0; x < 4; ++x) r.set(x, 1, 1.0f);  // row 1 complete
  r.set(0, 2, DemRaster::kNoData);                 // row 2 partially done
  r.set(2, 2, 5.0f);
  CHECK(r.todoInRow(0) == std::vector<int>{0, 1, 2, 3});
  CHECK(r.todoInRow(1).empty());
  CHECK(r.todoInRow(2) == std::vector<int>{1, 3});
  CHECK(r.rowsWithWork() == std::vector<int>{0, 2});
}

TEST_CASE("create rejects non-positive dimensions", "[demraster]") {
  CHECK_THROWS_AS(DemRaster::create(0, 3, kBox, {}), std::invalid_argument);
  CHECK_THROWS_AS(DemRaster::create(3, -1, kBox, {}), std::invalid_argument);
}

TEST_CASE("the image description round-trips and keeps the URL intact", "[demraster]") {
  const RasterMetadata meta{"Elevation_CCT_GLM2019_5m_DEM", "https://host.test/a;wms=b?x=1"};
  const std::string text = demdl::formatImageDescription(meta);
  CHECK(text == "dl_elevation;layer=Elevation_CCT_GLM2019_5m_DEM;wms=https://host.test/a;wms=b?x=1");
  const RasterMetadata back = demdl::parseImageDescription(text);
  CHECK(back.layer == meta.layer);
  CHECK(back.wmsUrl == meta.wmsUrl);
  CHECK(demdl::parseImageDescription("unrelated text").layer.empty());
  CHECK(demdl::parseImageDescription("unrelated text").wmsUrl.empty());
}

TEST_CASE("save and load round-trip values, NaN and NoData", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  r.set(1, 1, 12.5f);
  r.set(2, 0, DemRaster::kNoData);
  r.set(0, 2, -3.25f);

  r.save(file);

  CHECK(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.width() == 4);
  CHECK(back.height() == 3);
  CHECK(back.box().approxEquals(kBox, 1e-9));
  CHECK(back.at(1, 1) == 12.5f);
  CHECK(back.at(2, 0) == DemRaster::kNoData);
  CHECK(back.at(0, 2) == -3.25f);
  CHECK(std::isnan(back.at(0, 0)));
  CHECK(back.countNaN() == 9);
  CHECK(back.metadata().layer == "LayerA");
  CHECK(back.metadata().wmsUrl == "https://example.test/wms");
}

TEST_CASE("the file carries GeoTIFF keys, the tiepoint and the NoData tag", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  DemRaster::create(4, 3, kBox, {}).save(file);

  TIFF* tif = XTIFFOpen(file.string().c_str(), "r");
  REQUIRE(tif != nullptr);
  GTIF* gtif = GTIFNew(tif);
  REQUIRE(gtif != nullptr);

  unsigned short v = 0;
  CHECK(GTIFKeyGetSHORT(gtif, GTModelTypeGeoKey, &v, 0, 1) == 1);
  CHECK(v == ModelTypeGeographic);
  CHECK(GTIFKeyGetSHORT(gtif, GTRasterTypeGeoKey, &v, 0, 1) == 1);
  CHECK(v == RasterPixelIsArea);
  CHECK(GTIFKeyGetSHORT(gtif, GeographicTypeGeoKey, &v, 0, 1) == 1);
  CHECK(v == 4326);

  uint16_t count = 0;
  double* tie = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_GEOTIEPOINTS, &count, &tie) == 1);
  CHECK(count == 6);
  CHECK(tie[3] == 18.1);
  CHECK(tie[4] == -34.0);
  double* scale = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_GEOPIXELSCALE, &count, &scale) == 1);
  CHECK(count == 3);
  CHECK(scale[0] == (18.5 - 18.1) / 4.0);
  CHECK(scale[1] == (-34.0 - -34.3) / 3.0);

  char* nodata = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_GDAL_NODATA, &nodata) == 1);
  CHECK(std::string(nodata) == "-32768");
  uint16_t sampleFormat = 0;
  REQUIRE(TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &sampleFormat) == 1);
  CHECK(sampleFormat == SAMPLEFORMAT_IEEEFP);

  GTIFFree(gtif);
  XTIFFClose(tif);
}

TEST_CASE("matches accepts the same configuration and explains mismatches", "[demraster]") {
  const Config cfg = exampleConfig();
  const auto r = DemRaster::create(cfg.width, cfg.height, cfg.box, {cfg.layer, cfg.wmsUrl});
  std::string why;
  CHECK(r.matches(cfg, why));
  CHECK(why.empty());

  Config shifted = cfg;
  shifted.box.north += 0.01;
  CHECK_FALSE(r.matches(shifted, why));
  CHECK_THAT(why, ContainsSubstring("north: file -34.184541800, requested -34.174541800"));

  Config other = cfg;
  other.width = 100;
  other.layer = "Other";
  CHECK_FALSE(r.matches(other, why));
  CHECK_THAT(why, ContainsSubstring("width: file 2048, requested 100"));
  CHECK_THAT(why, ContainsSubstring("layer: file Elevation_CCT_GLM2019_5m_DEM, requested Other"));

  Config mirror = cfg;
  mirror.wmsUrl = "https://mirror.test/wms";
  CHECK(r.matches(mirror, why));
  CHECK_FALSE(r.wmsUrlWarning(mirror).empty());
  CHECK(r.wmsUrlWarning(cfg).empty());
}

TEST_CASE("legacy RGB16 files are rejected with a clear message", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "legacy.tif";
  {
    TIFF* tif = TIFFOpen(file.string().c_str(), "w");
    REQUIRE(tif != nullptr);
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 4u);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 2u);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 2u);
    std::vector<uint16_t> row(4 * 3, 0);
    for (uint32_t y = 0; y < 2; ++y) REQUIRE(TIFFWriteScanline(tif, row.data(), y, 0) >= 0);
    TIFFClose(tif);
  }
  CHECK_THROWS_WITH(DemRaster::load(file), ContainsSubstring("RGB16"));
}

TEST_CASE("a file with an unusable tiepoint is rejected", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "nan-tiepoint.tif";
  {
    TIFF* tif = XTIFFOpen(file.string().c_str(), "w");
    REQUIRE(tif != nullptr);
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 4u);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 2u);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
    TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 2u);
    const double tiepoints[6] = {0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), -34.0, 0.0};
    const double scale[3] = {0.1, 0.1, 0.0};
    REQUIRE(TIFFSetField(tif, TIFFTAG_GEOTIEPOINTS, 6, tiepoints) == 1);
    REQUIRE(TIFFSetField(tif, TIFFTAG_GEOPIXELSCALE, 3, scale) == 1);
    std::vector<float> row(4, 0.0f);
    for (uint32_t y = 0; y < 2; ++y) REQUIRE(TIFFWriteScanline(tif, row.data(), y, 0) >= 0);
    XTIFFClose(tif);
  }
  CHECK_THROWS_WITH(DemRaster::load(file), ContainsSubstring("invalid bounding box"));
}

TEST_CASE("loading a missing file reports the path", "[demraster]") {
  TempDir dir;
  CHECK_THROWS_WITH(DemRaster::load(dir.path / "absent.tif"), ContainsSubstring("absent.tif"));
}

namespace {

// Lowers this process's file-size limit, with SIGXFSZ ignored so that a write past the limit fails
// with EFBIG instead of killing the process. Restores both on destruction, however the scope ends.
class FileSizeLimit {
 public:
  explicit FileSizeLimit(std::uintmax_t bytes) {
    struct sigaction ignoreAction {};
    ignoreAction.sa_handler = SIG_IGN;
    sigemptyset(&ignoreAction.sa_mask);
    ignored_ = sigaction(SIGXFSZ, &ignoreAction, &previousAction_) == 0;
    if (ignored_ && getrlimit(RLIMIT_FSIZE, &previousLimit_) == 0) {
      rlimit lowered = previousLimit_;
      lowered.rlim_cur = static_cast<rlim_t>(bytes);
      lowered_ = setrlimit(RLIMIT_FSIZE, &lowered) == 0;
    }
  }
  ~FileSizeLimit() {
    if (lowered_) setrlimit(RLIMIT_FSIZE, &previousLimit_);
    if (ignored_) sigaction(SIGXFSZ, &previousAction_, nullptr);
  }
  FileSizeLimit(const FileSizeLimit&) = delete;
  FileSizeLimit& operator=(const FileSizeLimit&) = delete;

  bool active() const { return lowered_; }

 private:
  struct sigaction previousAction_ {};
  rlimit previousLimit_{};
  bool ignored_ = false;
  bool lowered_ = false;
};

struct LimitedSave {
  bool limited = false;  // the limit was in force during the save
  std::string error;     // empty when the save succeeded
};

// Saves r to file under a file-size limit. Nothing is reported while the limit is in force, so no
// test output can be cut short by it.
LimitedSave saveUnderLimit(const DemRaster& r, const std::filesystem::path& file, std::uintmax_t bytes) {
  LimitedSave result;
  const FileSizeLimit limit(bytes);
  result.limited = limit.active();
  try {
    r.save(file);
  } catch (const std::runtime_error& e) {
    result.error = e.what();
  }
  return result;
}

}  // namespace

TEST_CASE("a failed directory write throws and keeps the previous checkpoint", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  r.set(1, 1, 12.5f);
  r.save(file);

  // libtiff writes the directory after all strip data, so a limit one byte short of the complete file
  // admits the header and every strip but not the whole directory. The size is taken from a
  // successful save of exactly the content the failing save writes.
  r.set(1, 1, 99.0f);
  const auto probe = dir.path / "probe.tif";
  r.save(probe);
  const std::uintmax_t complete = std::filesystem::file_size(probe);

  const LimitedSave attempt = saveUnderLimit(r, file, complete - 1);
  REQUIRE(attempt.limited);
  CHECK_THAT(attempt.error, ContainsSubstring("cannot finish writing"));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.at(1, 1) == 12.5f);
}

TEST_CASE("a checkpoint that cannot be created leaves no temp file behind", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  r.set(1, 1, 12.5f);
  r.save(file);

  // With no room at all, libtiff creates and truncates the temp file, then fails to write its header.
  r.set(1, 1, 99.0f);
  const LimitedSave attempt = saveUnderLimit(r, file, 0);
  REQUIRE(attempt.limited);
  CHECK_THAT(attempt.error, ContainsSubstring("cannot create"));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.at(1, 1) == 12.5f);
}
