#pragma once

#include "demdl/Config.h"

// Today's Simon's Town run with the latitude names corrected (the old code
// stored the smaller latitude in a variable called "north").
inline demdl::Config exampleConfig() {
  demdl::Config c;
  c.box = {-34.1845418, -34.2163447, 18.4625244, 18.4185791};
  c.width = 2048;
  c.height = 1792;
  c.output = "simonstown.tif";
  return c;
}
