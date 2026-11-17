#pragma once

#include <stddef.h>
#include <stdint.h>

namespace robot_bridge {

constexpr uint16_t kMotorFrame = 0xABCD;
constexpr uint16_t kKettleFrame = 0xABCE;
constexpr size_t kFrameBytes = 8;

// Input and output use little-endian uint16_t fields, matching the existing
// Hoverboard command protocol. Never forward an unvalidated packet.
struct ParsedCommand {
  uint16_t start = 0;
  uint16_t word1 = 0;
  uint16_t word2 = 0;
  uint16_t checksum = 0;
  uint8_t bytes[kFrameBytes]{};
};

class CommandParser {
 public:
  // Called once per received byte, emits at most one validated 8-byte frame.
  bool feed(uint8_t byte, ParsedCommand& out) {
    if (length_ == kFrameBytes) dropFront();
    buffer_[length_++] = byte;
    for (;;) {
      if (length_ == 0) return false;
      if (buffer_[0] != 0xCD && buffer_[0] != 0xCE) {
        dropFront();
        continue;
      }
      if (length_ < 2) return false;
      if (buffer_[1] != 0xAB) {
        dropFront();
        continue;
      }
      if (length_ < kFrameBytes) return false;
      const uint16_t start = decode(0);
      const uint16_t word1 = decode(2);
      const uint16_t word2 = decode(4);
      const uint16_t checksum = decode(6);
      if ((start == kMotorFrame || start == kKettleFrame) &&
          checksum == static_cast<uint16_t>(start ^ word1 ^ word2)) {
        out.start = start;
        out.word1 = word1;
        out.word2 = word2;
        out.checksum = checksum;
        for (size_t i = 0; i < kFrameBytes; ++i) out.bytes[i] = buffer_[i];
        reset();
        return true;
      }
      // Recover after corruption without treating arbitrary USB read() chunks
      // as packets. Retain possible future start bytes during resynchronization.
      dropFront();
    }
  }

  void reset() { length_ = 0; }

 private:
  uint8_t buffer_[kFrameBytes]{};
  uint8_t length_ = 0;

  uint16_t decode(size_t offset) const {
    return static_cast<uint16_t>(buffer_[offset]) |
           (static_cast<uint16_t>(buffer_[offset + 1]) << 8);
  }

  void dropFront() {
    if (length_ == 0) return;
    for (uint8_t i = 1; i < length_; ++i) buffer_[i - 1] = buffer_[i];
    --length_;
  }
};

}  // namespace robot_bridge
