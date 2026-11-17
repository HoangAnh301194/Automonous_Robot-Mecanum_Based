// Run from firmware/esp32_bridge:
//   g++ -std=c++17 -Wall -Wextra -Werror -I src tools/test_command_parser.cpp -o /tmp/test_command_parser
//   /tmp/test_command_parser
#include "command_parser.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>
using namespace robot_bridge;

std::vector<uint8_t> frame(uint16_t start, uint16_t w1, uint16_t w2) {
  const uint16_t c = start ^ w1 ^ w2;
  return {static_cast<uint8_t>(start), static_cast<uint8_t>(start >> 8),
          static_cast<uint8_t>(w1), static_cast<uint8_t>(w1 >> 8),
          static_cast<uint8_t>(w2), static_cast<uint8_t>(w2 >> 8),
          static_cast<uint8_t>(c), static_cast<uint8_t>(c >> 8)};
}

int main() {
  CommandParser parser;
  ParsedCommand out;
  auto motor = frame(kMotorFrame, 100, static_cast<uint16_t>(-100));
  auto kettle = frame(kKettleFrame, 2, 0);
  unsigned got = 0;
  for (uint8_t byte : motor) {
    if (parser.feed(byte, out)) {
      assert(out.start == kMotorFrame);
      assert(out.word1 == 100);
      assert(out.word2 == static_cast<uint16_t>(-100));
      ++got;
    }
  }
  for (uint8_t byte : kettle) {
    if (parser.feed(byte, out)) {
      assert(out.start == kKettleFrame);
      assert(out.word1 == 2);
      ++got;
    }
  }
  assert(got == 2);

  auto damaged = motor;
  damaged[3] ^= 1;
  got = 0;
  for (uint8_t byte : damaged) if (parser.feed(byte, out)) ++got;
  assert(got == 0);

  std::vector<uint8_t> stream = {0xff, 0, 0xcd, 0xff, 0xce};
  stream.insert(stream.end(), damaged.begin(), damaged.end());
  stream.insert(stream.end(), motor.begin(), motor.end());
  stream.insert(stream.end(), kettle.begin(), kettle.end());
  got = 0;
  parser.reset();
  for (uint8_t byte : stream) {
    if (parser.feed(byte, out)) {
      ++got;
      assert(out.start == (got == 1 ? kMotorFrame : kKettleFrame));
    }
  }
  assert(got == 2);

  parser.reset();
  for (size_t i = 0; i < 5; ++i) assert(!parser.feed(motor[i], out));
  parser.reset();
  got = 0;
  for (uint8_t byte : kettle) {
    if (parser.feed(byte, out)) {
      assert(out.start == kKettleFrame);
      ++got;
    }
  }
  assert(got == 1);
  std::cout << "PASS: motor, kettle, checksum, resync, fragmentation, reset\n";
}
