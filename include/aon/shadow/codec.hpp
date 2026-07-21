#pragma once

#include "aon/shadow/processor.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace aon::shadow {

constexpr std::size_t kMaximumEncodedBytes = 256U * 1024U;

struct EncodedRecording {
  std::array<std::uint8_t, kMaximumEncodedBytes> data{};
  std::size_t size = 0;
};

struct DecodedRecording {
  std::uint32_t generation = 0;
  Capture capture{};
  ProcessedRoute route{};
};

struct GenerationInfo {
  bool valid = false;
  std::uint32_t generation = 0;
};

enum class Generation : std::uint8_t { None, A, B };

ResultCode encode(const Capture& capture, const ProcessedRoute& route,
                  std::uint32_t generation, EncodedRecording& out);
ResultCode decode(const std::uint8_t* data, std::size_t size,
                  RobotIdentity expectedRobot, DecodedRecording& out);
Generation chooseGeneration(GenerationInfo a, GenerationInfo b);

}  // namespace aon::shadow
