// Serial command input: line assembly and "name value" parsing.
#include <string.h>

#include "check.h"
#include "command.h"

namespace {

LineStatus feed(LineReader& reader, const char* text) {
  LineStatus last = LineStatus::Pending;
  for (const char* p = text; *p; ++p) {
    const LineStatus status = reader.push(*p);
    if (status != LineStatus::Pending) last = status;
  }
  return last;
}

}  // namespace

TEST(line_is_ready_only_after_newline) {
  LineReader reader;
  CHECK(feed(reader, "kp 15") == LineStatus::Pending);
  CHECK(reader.push('\n') == LineStatus::Ready);
  CHECK(strcmp(reader.line(), "kp 15") == 0);
}

TEST(crlf_produces_a_single_line) {
  LineReader reader;
  int ready = 0;
  for (const char* p = "save\r\n"; *p; ++p) {
    if (reader.push(*p) == LineStatus::Ready) ++ready;
  }
  CHECK(ready == 1);
  CHECK(strcmp(reader.line(), "save") == 0);
}

TEST(next_line_starts_fresh_after_ready) {
  LineReader reader;
  feed(reader, "kp 15\n");
  CHECK(feed(reader, "ki 1\n") == LineStatus::Ready);
  CHECK(strcmp(reader.line(), "ki 1") == 0);
}

TEST(too_long_line_is_reported_and_dropped) {
  LineReader reader;
  for (size_t i = 0; i < LineReader::kMaxLength + 5; ++i) reader.push('x');
  CHECK(reader.push('\n') == LineStatus::Overflow);
  CHECK(feed(reader, "kd 1\n") == LineStatus::Ready);
  CHECK(strcmp(reader.line(), "kd 1") == 0);
}

TEST(parses_original_one_letter_form) {
  Command cmd;
  CHECK(parseCommand("p15.5", cmd) == ParseStatus::Ok);
  CHECK(strcmp(cmd.name, "p") == 0);
  CHECK(cmd.hasValue);
  CHECK_NEAR(cmd.value, 15.5, 1e-6);
}

TEST(parses_name_space_and_equals_forms) {
  Command cmd;
  CHECK(parseCommand("  KP = -2.5 ", cmd) == ParseStatus::Ok);
  CHECK(strcmp(cmd.name, "kp") == 0);
  CHECK_NEAR(cmd.value, -2.5, 1e-6);
  CHECK(parseCommand("turnramp 1.2", cmd) == ParseStatus::Ok);
  CHECK(strcmp(cmd.name, "turnramp") == 0);
  CHECK_NEAR(cmd.value, 1.2, 1e-6);
}

TEST(parses_command_without_value) {
  Command cmd;
  CHECK(parseCommand("save", cmd) == ParseStatus::Ok);
  CHECK(strcmp(cmd.name, "save") == 0);
  CHECK(!cmd.hasValue);
  CHECK(parseCommand("?", cmd) == ParseStatus::Ok);
  CHECK(strcmp(cmd.name, "?") == 0);
}

TEST(rejects_garbage_values_and_names) {
  Command cmd;
  CHECK(parseCommand("kp 1x", cmd) == ParseStatus::BadValue);
  CHECK(parseCommand("kp nan", cmd) == ParseStatus::BadValue);
  CHECK(parseCommand("15", cmd) == ParseStatus::BadName);
  CHECK(parseCommand("   ", cmd) == ParseStatus::Empty);
  CHECK(parseCommand("averyveryverylongname 1", cmd) == ParseStatus::BadName);
}
