// =============================================================================
// Text commands: collecting characters into lines and parsing "name value".
// Pure C++ (no Arduino dependencies) so it is unit-tested on the host.
// =============================================================================
#pragma once

#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

enum class LineStatus { Pending, Ready, Overflow };

// Collects characters into lines without ever waiting for more input, so the
// balance loop never stalls while someone is typing.
class LineReader {
 public:
  static constexpr size_t kMaxLength = 63;

  LineStatus push(char c) {
    if (lineReady_) {
      length_ = 0;
      lineReady_ = false;
    }
    if (c == '\n' || c == '\r') {
      const bool overflowed = overflow_;
      const bool hasText = length_ > 0;
      overflow_ = false;
      if (overflowed) {
        length_ = 0;
        return LineStatus::Overflow;
      }
      if (!hasText) return LineStatus::Pending;  // empty line or the '\n' of "\r\n"
      buffer_[length_] = '\0';
      lineReady_ = true;
      return LineStatus::Ready;
    }
    if (length_ < kMaxLength) {
      buffer_[length_++] = c;
    } else {
      overflow_ = true;
    }
    return LineStatus::Pending;
  }

  // Valid after push() returned Ready, until the next push().
  const char* line() const { return buffer_; }

 private:
  char buffer_[kMaxLength + 1] = {};
  size_t length_ = 0;
  bool overflow_ = false;
  bool lineReady_ = false;
};

struct Command {
  static constexpr size_t kMaxName = 15;
  char name[kMaxName + 1] = {};
  bool hasValue = false;
  float value = 0.0f;
};

enum class ParseStatus { Ok, Empty, BadName, BadValue };

// Parses "kp 15", "kp=15", "p15", "save" or "?". Names are lower-cased.
inline ParseStatus parseCommand(const char* text, Command& out) {
  out = Command();
  const char* p = text;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '\0') return ParseStatus::Empty;

  size_t length = 0;
  while (isalpha(static_cast<unsigned char>(*p)) || *p == '?') {
    if (length == Command::kMaxName) return ParseStatus::BadName;
    out.name[length++] = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
    ++p;
  }
  out.name[length] = '\0';
  if (length == 0) return ParseStatus::BadName;

  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '=') ++p;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '\0') return ParseStatus::Ok;

  char* end = nullptr;
  const float value = strtof(p, &end);
  if (end == p || !isfinite(value)) return ParseStatus::BadValue;
  while (*end == ' ' || *end == '\t') ++end;
  if (*end != '\0') return ParseStatus::BadValue;

  out.hasValue = true;
  out.value = value;
  return ParseStatus::Ok;
}
