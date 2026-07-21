#include "aon/shadow/codec.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace aon::shadow {
namespace {

constexpr std::uint8_t kMagic[] = {'A', 'O', 'N', 'S', 'H', 'D', 'W', '1'};
constexpr std::uint16_t kFormatVersion = 1;
constexpr std::uint16_t kHeaderSize = 69;
constexpr std::size_t kCrcOffset = 65;
constexpr std::uint32_t kCrcPolynomial = 0xEDB88320U;
constexpr std::uint32_t kSampleBytes = 24;
constexpr std::uint32_t kEventBytes = 7;
constexpr std::uint32_t kSegmentBytes = 10;
constexpr std::uint32_t kPointBytes = 12;
constexpr std::uint32_t kAnchoredEventBytes = 17;

static_assert(sizeof(float) == sizeof(std::uint32_t),
              "32-bit floats required");
static_assert(std::numeric_limits<float>::is_iec559,
              "IEEE-754 floating point required");
static_assert(std::numeric_limits<float>::digits == 24,
              "IEEE-754 binary32 precision required");
static_assert(std::numeric_limits<float>::max_exponent == 128,
              "IEEE-754 binary32 exponent range required");

std::int8_t decodeI8(std::uint8_t value) {
  if (value <= static_cast<std::uint8_t>(
                   std::numeric_limits<std::int8_t>::max())) {
    return static_cast<std::int8_t>(value);
  }
  return static_cast<std::int8_t>(
      -1 - static_cast<std::int16_t>(0xFFU - value));
}

std::int16_t decodeI16(std::uint16_t value) {
  if (value <= static_cast<std::uint16_t>(
                   std::numeric_limits<std::int16_t>::max())) {
    return static_cast<std::int16_t>(value);
  }
  return static_cast<std::int16_t>(
      -1 - static_cast<std::int32_t>(0xFFFFU - value));
}

class Writer {
 public:
  explicit Writer(EncodedRecording& output) : output_(output) {
    output_.size = 0;
  }

  bool byte(std::uint8_t value) {
    if (output_.size == output_.data.size()) return false;
    output_.data[output_.size++] = value;
    return true;
  }

  bool u16(std::uint16_t value) {
    return byte(static_cast<std::uint8_t>(value)) &&
           byte(static_cast<std::uint8_t>(value >> 8U));
  }

  bool u32(std::uint32_t value) {
    for (std::size_t index = 0; index < 4; ++index) {
      if (!byte(static_cast<std::uint8_t>(value >> (index * 8U)))) return false;
    }
    return true;
  }

  bool f32(float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return u32(bits);
  }

 private:
  EncodedRecording& output_;
};

class Reader {
 public:
  Reader(const std::uint8_t* data, std::size_t size, std::size_t offset = 0)
      : data_(data), size_(size), offset_(offset) {}

  bool byte(std::uint8_t& value) {
    if (offset_ >= size_) return false;
    value = data_[offset_++];
    return true;
  }

  bool u16(std::uint16_t& value) {
    std::uint8_t low = 0, high = 0;
    if (!byte(low) || !byte(high)) return false;
    value = static_cast<std::uint16_t>(low) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(high) << 8U);
    return true;
  }

  bool u32(std::uint32_t& value) {
    value = 0;
    for (std::size_t index = 0; index < 4; ++index) {
      std::uint8_t part = 0;
      if (!byte(part)) return false;
      value |= static_cast<std::uint32_t>(part) << (index * 8U);
    }
    return true;
  }

  bool f32(float& value) {
    std::uint32_t bits = 0;
    if (!u32(bits)) return false;
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  std::size_t offset() const { return offset_; }

 private:
  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t offset_;
};

bool validRobot(RobotIdentity robot) {
  return robot == RobotIdentity::Small || robot == RobotIdentity::Big;
}

bool validDirection(Direction direction) {
  return direction == Direction::Reverse || direction == Direction::Stopped ||
         direction == Direction::Forward;
}

bool validMechanism(MechanismKind kind) {
  return static_cast<std::uint8_t>(kind) <=
         static_cast<std::uint8_t>(MechanismKind::Arrow);
}

bool finite(float value) { return std::isfinite(value); }

bool validate(const Capture& capture, const ProcessedRoute& route) {
  if (!validRobot(capture.robot) || capture.sampleCount > kMaximumSamples ||
      capture.eventCount > kMaximumEvents ||
      route.segmentCount > kMaximumSegments ||
      route.pointCount > kMaximumPathPoints ||
      route.eventCount > kMaximumEvents ||
      capture.durationMs > kMaximumDurationMs || route.result != ResultCode::Ok ||
      !finite(route.start.x) || !finite(route.start.y) ||
      !finite(route.start.heading)) {
    return false;
  }

  for (std::size_t index = 0; index < capture.sampleCount; ++index) {
    const auto& sample = capture.samples[index];
    if (!finite(sample.x) || !finite(sample.y) || !finite(sample.heading) ||
        !validDirection(sample.direction) || sample.timeMs > capture.durationMs ||
        (index > 0 && sample.timeMs <= capture.samples[index - 1].timeMs)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < capture.eventCount; ++index) {
    const auto& event = capture.events[index];
    if (!validMechanism(event.kind) || event.timeMs > capture.durationMs ||
        (index > 0 && event.timeMs < capture.events[index - 1].timeMs)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < route.segmentCount; ++index) {
    const auto& segment = route.segments[index];
    const auto rangeEnd = static_cast<std::size_t>(segment.firstPoint) +
                          static_cast<std::size_t>(segment.pointCount);
    if ((segment.kind != SegmentKind::Motion &&
         segment.kind != SegmentKind::Dwell) ||
        !validDirection(segment.direction) || rangeEnd > route.pointCount) {
      return false;
    }
  }
  for (std::size_t index = 0; index < route.pointCount; ++index) {
    const auto& point = route.points[index];
    if (!finite(point.x) || !finite(point.y) || !finite(point.speed)) return false;
  }
  for (std::size_t index = 0; index < route.eventCount; ++index) {
    const auto& event = route.events[index];
    if (!validMechanism(event.event.kind) ||
        event.event.timeMs > capture.durationMs ||
        event.segmentIndex >= route.segmentCount || !finite(event.progress) ||
        event.progress < 0.0F || event.progress > 1.0F ||
        event.offsetMs > route.segments[event.segmentIndex].durationMs ||
        (index > 0 && event.event.timeMs < route.events[index - 1].event.timeMs)) {
      return false;
    }
  }
  return true;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (std::uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ ((crc & 1U) ? kCrcPolynomial : 0U);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

bool writeEvent(Writer& writer, const MechanismEvent& event) {
  return writer.u32(event.timeMs) &&
         writer.byte(static_cast<std::uint8_t>(event.kind)) &&
         writer.u16(static_cast<std::uint16_t>(event.value));
}

bool readEvent(Reader& reader, MechanismEvent& event) {
  std::uint8_t kind = 0;
  std::uint16_t value = 0;
  if (!reader.u32(event.timeMs) || !reader.byte(kind) || !reader.u16(value)) {
    return false;
  }
  event.kind = static_cast<MechanismKind>(kind);
  event.value = decodeI16(value);
  return true;
}

}  // namespace

ResultCode encode(const Capture& capture, const ProcessedRoute& route,
                  std::uint32_t generation, EncodedRecording& out) {
  if (!validate(capture, route)) return ResultCode::CorruptFile;

  const auto sampleCount = static_cast<std::uint16_t>(capture.sampleCount);
  const auto rawEventCount = static_cast<std::uint16_t>(capture.eventCount);
  const auto segmentCount = static_cast<std::uint16_t>(route.segmentCount);
  const auto pointCount = static_cast<std::uint16_t>(route.pointCount);
  const auto anchoredCount = static_cast<std::uint16_t>(route.eventCount);
  const std::uint32_t sectionSizes[] = {
      static_cast<std::uint32_t>(sampleCount) * kSampleBytes,
      static_cast<std::uint32_t>(rawEventCount) * kEventBytes,
      static_cast<std::uint32_t>(segmentCount) * kSegmentBytes,
      static_cast<std::uint32_t>(pointCount) * kPointBytes,
      static_cast<std::uint32_t>(anchoredCount) * kAnchoredEventBytes};

  Writer writer(out);
  for (const auto byte : kMagic) if (!writer.byte(byte)) return ResultCode::WriteFailed;
  if (!writer.u16(kFormatVersion) || !writer.u16(kHeaderSize) ||
      !writer.u32(generation) ||
      !writer.byte(static_cast<std::uint8_t>(capture.robot)) ||
      !writer.u16(static_cast<std::uint16_t>(kSamplePeriodMs)) ||
      !writer.u32(capture.durationMs) || !writer.f32(route.start.x) ||
      !writer.f32(route.start.y) || !writer.f32(route.start.heading) ||
      !writer.u16(sampleCount) || !writer.u16(rawEventCount) ||
      !writer.u16(segmentCount) || !writer.u16(pointCount) ||
      !writer.u16(anchoredCount)) {
    return ResultCode::WriteFailed;
  }
  for (const auto size : sectionSizes) {
    if (!writer.u32(size)) return ResultCode::WriteFailed;
  }
  if (!writer.u32(0)) return ResultCode::WriteFailed;

  for (std::size_t index = 0; index < capture.sampleCount; ++index) {
    const auto& sample = capture.samples[index];
    if (!writer.u32(sample.timeMs) || !writer.f32(sample.x) ||
        !writer.f32(sample.y) || !writer.f32(sample.heading) ||
        !writer.byte(static_cast<std::uint8_t>(sample.leftX)) ||
        !writer.byte(static_cast<std::uint8_t>(sample.leftY)) ||
        !writer.byte(static_cast<std::uint8_t>(sample.rightX)) ||
        !writer.byte(static_cast<std::uint8_t>(sample.rightY)) ||
        !writer.byte(static_cast<std::uint8_t>(sample.leftCommand)) ||
        !writer.byte(static_cast<std::uint8_t>(sample.rightCommand)) ||
        !writer.byte(static_cast<std::uint8_t>(sample.direction)) ||
        !writer.byte(sample.poseValid ? 1U : 0U)) return ResultCode::WriteFailed;
  }
  for (std::size_t index = 0; index < capture.eventCount; ++index) {
    if (!writeEvent(writer, capture.events[index])) return ResultCode::WriteFailed;
  }
  for (std::size_t index = 0; index < route.segmentCount; ++index) {
    const auto& segment = route.segments[index];
    if (!writer.byte(static_cast<std::uint8_t>(segment.kind)) ||
        !writer.byte(static_cast<std::uint8_t>(segment.direction)) ||
        !writer.u16(segment.firstPoint) || !writer.u16(segment.pointCount) ||
        !writer.u32(segment.durationMs)) return ResultCode::WriteFailed;
  }
  for (std::size_t index = 0; index < route.pointCount; ++index) {
    const auto& point = route.points[index];
    if (!writer.f32(point.x) || !writer.f32(point.y) || !writer.f32(point.speed)) {
      return ResultCode::WriteFailed;
    }
  }
  for (std::size_t index = 0; index < route.eventCount; ++index) {
    const auto& event = route.events[index];
    if (!writeEvent(writer, event.event) || !writer.u16(event.segmentIndex) ||
        !writer.f32(event.progress) || !writer.u32(event.offsetMs)) {
      return ResultCode::WriteFailed;
    }
  }

  const auto checksum = crc32(out.data.data() + kHeaderSize,
                              out.size - kHeaderSize);
  for (std::size_t index = 0; index < 4; ++index) {
    out.data[kCrcOffset + index] =
        static_cast<std::uint8_t>(checksum >> (index * 8U));
  }
  return ResultCode::Ok;
}

ResultCode decode(const std::uint8_t* data, std::size_t size,
                  RobotIdentity expectedRobot, DecodedRecording& out) {
  if (data == nullptr || size < kHeaderSize || size > kMaximumEncodedBytes) {
    return ResultCode::CorruptFile;
  }
  Reader reader(data, size);
  for (const auto expected : kMagic) {
    std::uint8_t actual = 0;
    if (!reader.byte(actual) || actual != expected) return ResultCode::CorruptFile;
  }
  std::uint16_t version = 0, headerSize = 0, samplePeriod = 0;
  std::uint32_t generation = 0, duration = 0;
  std::uint8_t robotValue = 0;
  float startX = 0, startY = 0, startHeading = 0;
  std::uint16_t counts[5]{};
  std::uint32_t sectionSizes[5]{};
  std::uint32_t expectedCrc = 0;
  if (!reader.u16(version)) return ResultCode::CorruptFile;
  if (version != kFormatVersion) return ResultCode::UnsupportedVersion;
  if (!reader.u16(headerSize) || headerSize != kHeaderSize ||
      !reader.u32(generation) || !reader.byte(robotValue) ||
      !reader.u16(samplePeriod) || !reader.u32(duration) ||
      !reader.f32(startX) || !reader.f32(startY) ||
      !reader.f32(startHeading)) return ResultCode::CorruptFile;
  for (auto& count : counts) if (!reader.u16(count)) return ResultCode::CorruptFile;
  for (auto& sectionSize : sectionSizes) {
    if (!reader.u32(sectionSize)) return ResultCode::CorruptFile;
  }
  if (!reader.u32(expectedCrc) || reader.offset() != kHeaderSize) {
    return ResultCode::CorruptFile;
  }

  const auto robot = static_cast<RobotIdentity>(robotValue);
  if (!validRobot(robot)) return ResultCode::CorruptFile;
  if (robot != expectedRobot) return ResultCode::WrongRobot;
  if (samplePeriod != kSamplePeriodMs || duration > kMaximumDurationMs ||
      !finite(startX) || !finite(startY) || !finite(startHeading) ||
      counts[0] > kMaximumSamples || counts[1] > kMaximumEvents ||
      counts[2] > kMaximumSegments || counts[3] > kMaximumPathPoints ||
      counts[4] > kMaximumEvents) return ResultCode::CorruptFile;

  const std::uint32_t expectedSizes[] = {
      static_cast<std::uint32_t>(counts[0]) * kSampleBytes,
      static_cast<std::uint32_t>(counts[1]) * kEventBytes,
      static_cast<std::uint32_t>(counts[2]) * kSegmentBytes,
      static_cast<std::uint32_t>(counts[3]) * kPointBytes,
      static_cast<std::uint32_t>(counts[4]) * kAnchoredEventBytes};
  std::size_t declaredSize = kHeaderSize;
  for (std::size_t index = 0; index < 5; ++index) {
    if (sectionSizes[index] != expectedSizes[index]) return ResultCode::CorruptFile;
    declaredSize += sectionSizes[index];
  }
  if (declaredSize != size ||
      crc32(data + kHeaderSize, size - kHeaderSize) != expectedCrc) {
    return ResultCode::CorruptFile;
  }

  out.generation = generation;
  out.capture.robot = robot;
  out.capture.durationMs = duration;
  out.capture.sampleCount = counts[0];
  out.capture.eventCount = counts[1];
  out.route.result = ResultCode::Ok;
  out.route.start = {};
  out.route.start.x = startX;
  out.route.start.y = startY;
  out.route.start.heading = startHeading;
  out.route.start.poseValid = true;
  out.route.segmentCount = counts[2];
  out.route.pointCount = counts[3];
  out.route.eventCount = counts[4];

  Reader payload(data, size, kHeaderSize);
  for (std::size_t index = 0; index < out.capture.sampleCount; ++index) {
    auto& sample = out.capture.samples[index];
    std::uint8_t leftX = 0, leftY = 0, rightX = 0, rightY = 0;
    std::uint8_t leftCommand = 0, rightCommand = 0, direction = 0, poseValid = 0;
    if (!payload.u32(sample.timeMs) || !payload.f32(sample.x) ||
        !payload.f32(sample.y) || !payload.f32(sample.heading) ||
        !payload.byte(leftX) || !payload.byte(leftY) ||
        !payload.byte(rightX) || !payload.byte(rightY) ||
        !payload.byte(leftCommand) || !payload.byte(rightCommand) ||
        !payload.byte(direction) || !payload.byte(poseValid) || poseValid > 1U) {
      return ResultCode::CorruptFile;
    }
    sample.leftX = decodeI8(leftX);
    sample.leftY = decodeI8(leftY);
    sample.rightX = decodeI8(rightX);
    sample.rightY = decodeI8(rightY);
    sample.leftCommand = decodeI8(leftCommand);
    sample.rightCommand = decodeI8(rightCommand);
    sample.direction = static_cast<Direction>(decodeI8(direction));
    sample.poseValid = poseValid != 0;
  }
  for (std::size_t index = 0; index < out.capture.eventCount; ++index) {
    if (!readEvent(payload, out.capture.events[index])) return ResultCode::CorruptFile;
  }
  for (std::size_t index = 0; index < out.route.segmentCount; ++index) {
    auto& segment = out.route.segments[index];
    std::uint8_t kind = 0, direction = 0;
    if (!payload.byte(kind) || !payload.byte(direction) ||
        !payload.u16(segment.firstPoint) || !payload.u16(segment.pointCount) ||
        !payload.u32(segment.durationMs)) return ResultCode::CorruptFile;
    segment.kind = static_cast<SegmentKind>(kind);
    segment.direction = static_cast<Direction>(decodeI8(direction));
  }
  for (std::size_t index = 0; index < out.route.pointCount; ++index) {
    auto& point = out.route.points[index];
    if (!payload.f32(point.x) || !payload.f32(point.y) ||
        !payload.f32(point.speed)) return ResultCode::CorruptFile;
  }
  for (std::size_t index = 0; index < out.route.eventCount; ++index) {
    auto& event = out.route.events[index];
    if (!readEvent(payload, event.event) || !payload.u16(event.segmentIndex) ||
        !payload.f32(event.progress) || !payload.u32(event.offsetMs)) {
      return ResultCode::CorruptFile;
    }
  }
  if (payload.offset() != size || !validate(out.capture, out.route)) {
    return ResultCode::CorruptFile;
  }
  return ResultCode::Ok;
}

Generation chooseGeneration(GenerationInfo a, GenerationInfo b) {
  if (!a.valid) return b.valid ? Generation::B : Generation::None;
  if (!b.valid) return Generation::A;
  return b.generation > a.generation ? Generation::B : Generation::A;
}

}  // namespace aon::shadow
