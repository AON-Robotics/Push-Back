#pragma once

#include <algorithm>
#include <cstdint>

namespace aon::lemlib_integration {

struct DriveCommand {
  std::int8_t left = 0;
  std::int8_t right = 0;
};

constexpr std::uint16_t packDriveCommand(int left, int right) {
  const auto byte = [](int value) {
    return static_cast<std::uint8_t>(std::clamp(value, -127, 127));
  };
  return static_cast<std::uint16_t>(byte(left)) |
         static_cast<std::uint16_t>(byte(right)) << 8U;
}

constexpr DriveCommand unpackDriveCommand(std::uint16_t packed) {
  const auto signedByte = [](std::uint8_t value) {
    const int decoded = value <= 127 ? value : static_cast<int>(value) - 256;
    return static_cast<std::int8_t>(decoded);
  };
  return {
      signedByte(static_cast<std::uint8_t>(packed & 0xffU)),
      signedByte(static_cast<std::uint8_t>((packed >> 8U) & 0xffU)),
  };
}

}  // namespace aon::lemlib_integration
