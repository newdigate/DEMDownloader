#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <string>

#include "app/CliChecks.h"

using Catch::Matchers::EndsWith;
using demdl::cli::checkNotFlagLike;
using demdl::cli::checkWholeNumber;
using demdl::cli::isPlainDecimal;
using demdl::cli::isPlainInteger;
using demdl::cli::quoted;

TEST_CASE("isPlainDecimal accepts a sign, a bare or full fraction and an exponent", "[clichecks]") {
  const std::string text = GENERATE(as<std::string>{}, "12", "12.", ".5", "-34.1845418", "1e-3", "+.5");
  CAPTURE(text);
  CHECK(isPlainDecimal(text));
}

TEST_CASE("isPlainDecimal refuses empty text, separators, hex floats, nan, a bare exponent and trailing letters",
          "[clichecks]") {
  const std::string text = GENERATE(as<std::string>{}, "", "1,5", "0x1p3", "nan", "1e", "12abc");
  CAPTURE(text);
  CHECK_FALSE(isPlainDecimal(text));
}

TEST_CASE("isPlainInteger accepts zero and whole numbers without leading zeros", "[clichecks]") {
  const std::string text = GENERATE(as<std::string>{}, "0", "7", "-5", "2048");
  CAPTURE(text);
  CHECK(isPlainInteger(text));
}

TEST_CASE("isPlainInteger refuses minus zero, leading zeros, prefixes, separators, words and empty text",
          "[clichecks]") {
  const std::string text = GENERATE(as<std::string>{}, "-0", "0100", "0x10", "1_000", "true", "");
  CAPTURE(text);
  CHECK_FALSE(isPlainInteger(text));
}

TEST_CASE("quoted shows a carriage return and bytes outside printable ASCII as escapes", "[clichecks]") {
  CHECK(quoted("a\rb") == "'a\\rb'");                      // a backslash and an r, not a carriage return
  CHECK(quoted("\xE2\x88\x92") == "'\\xE2\\x88\\x92'");  // a Unicode minus sign
}

TEST_CASE("checkWholeNumber ends with the refused value quoted", "[clichecks]") {
  CHECK_THAT(checkWholeNumber("0100"), EndsWith(", not '0100'"));
}

TEST_CASE("checkNotFlagLike refuses a value that begins with two dashes", "[clichecks]") {
  CHECK(checkNotFlagLike("--x") == "looks like a flag; is the value missing?");
}
