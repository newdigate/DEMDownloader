#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "demdl/DemRaster.h"

using demdl::BoundingBox;
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
  const RasterMetadata meta{"Elevation_CCT_GLM2019_5m_DEM", "https://host.test/a;b=c?x=1"};
  const std::string text = demdl::formatImageDescription(meta);
  CHECK(text == "dl_elevation;layer=Elevation_CCT_GLM2019_5m_DEM;wms=https://host.test/a;b=c?x=1");
  const RasterMetadata back = demdl::parseImageDescription(text);
  CHECK(back.layer == meta.layer);
  CHECK(back.wmsUrl == meta.wmsUrl);
  CHECK(demdl::parseImageDescription("unrelated text").layer.empty());
  CHECK(demdl::parseImageDescription("unrelated text").wmsUrl.empty());
}
