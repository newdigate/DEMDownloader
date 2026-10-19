#pragma once

#include <cstddef>
#include <string>

namespace demdl::cli {

// The strict checks on option text that the command-line parsers run before CLI11 converts it.

namespace detail {

// The character tests are ASCII only, so the result does not depend on the locale.
inline bool isDigit(char c) { return c >= '0' && c <= '9'; }

// Moves i past a run of digits and returns how many there were.
inline std::size_t skipDigits(const std::string& text, std::size_t& i) {
  const std::size_t start = i;
  while (i < text.size() && isDigit(text[i])) ++i;
  return i - start;
}

}  // namespace detail

// An optional sign, digits with an optional fraction or a fraction alone, then an optional exponent:
// 12, 12., .5, -34.1845418, 1e-3.
[[nodiscard]] inline bool isPlainDecimal(const std::string& text) {
  std::size_t i = 0;
  if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
  std::size_t digits = detail::skipDigits(text, i);
  if (i < text.size() && text[i] == '.') {
    ++i;
    digits += detail::skipDigits(text, i);
  }
  if (digits == 0) return false;
  if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
    ++i;
    if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
    if (detail::skipDigits(text, i) == 0) return false;
  }
  return i == text.size();
}

// An optional '-', then digits without a leading zero; "0" itself is allowed.
[[nodiscard]] inline bool isPlainInteger(const std::string& text) {
  if (text == "0") return true;
  const std::size_t first = text.starts_with('-') ? 1 : 0;
  if (first == text.size() || text[first] == '0') return false;
  for (std::size_t i = first; i < text.size(); ++i) {
    if (!detail::isDigit(text[i])) return false;
  }
  return true;
}

// `text` in single quotes, readable whatever it holds: \r, \n and \t become those escapes, a backslash
// becomes \\, a single quote \', and any other byte outside printable ASCII (0x20..0x7E) \xHH.
[[nodiscard]] inline std::string quoted(const std::string& text) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string out = "'";
  for (const char c : text) {
    const auto byte = static_cast<unsigned char>(c);
    switch (c) {
      case '\r': out += "\\r"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      case '\\': out += "\\\\"; break;
      case '\'': out += "\\'"; break;
      default:
        if (byte >= 0x20 && byte <= 0x7E) {
          out += c;
        } else {
          out += "\\x";
          out += kHex[byte >> 4];
          out += kHex[byte & 0x0F];
        }
    }
  }
  return out + "'";
}

// The checks below return an empty string for an acceptable value, otherwise the problem. The numeric ones end
// with the value quoted, so that stray characters such as a trailing space or a carriage return show.

[[nodiscard]] inline std::string checkDecimal(const std::string& text) {
  if (isPlainDecimal(text)) return {};
  return "must be a decimal number such as -34.1845418 (use '.' as the decimal separator), not " + quoted(text);
}

[[nodiscard]] inline std::string checkWholeNumber(const std::string& text) {
  if (isPlainInteger(text)) return {};
  return "must be a whole number without leading zeros, separators or prefixes, not " + quoted(text);
}

[[nodiscard]] inline std::string checkNotFlagLike(const std::string& text) {
  if (!text.starts_with("--")) return {};
  return "looks like a flag; is the value missing?";
}

}  // namespace demdl::cli
