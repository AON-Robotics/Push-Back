#include "aon/shadow/storage.hpp"

#include <cstdio>
#include <limits>

namespace aon::shadow {
namespace {

struct Candidate {
  ResultCode result = ResultCode::EmptyRecording;
  bool valid = false;
  std::uint32_t generation = 0;
};

bool validSlot(std::uint8_t slot) { return slot >= 1 && slot <= kSlotCount; }

void pathFor(std::uint8_t slot, char generation, char* path,
             std::size_t pathSize) {
  std::snprintf(path, pathSize, "/usd/aon-shadow-slot-%u-%c.bin",
                static_cast<unsigned>(slot), generation);
}

Candidate readCandidate(const FileStore& files, const char* path,
                        RobotIdentity robot, EncodedRecording& encoded,
                        DecodedRecording& decoded) {
  const ResultCode readResult = files.read(path, encoded);
  if (readResult != ResultCode::Ok) return {readResult, false, 0};
  const ResultCode decodeResult =
      decode(encoded.data.data(), encoded.size, robot, decoded);
  return {decodeResult, decodeResult == ResultCode::Ok, decoded.generation};
}

ResultCode failureFor(const Candidate& a, const Candidate& b) {
  if (a.result == ResultCode::WrongRobot || b.result == ResultCode::WrongRobot) {
    return ResultCode::WrongRobot;
  }
  if (a.result == ResultCode::UnsupportedVersion ||
      b.result == ResultCode::UnsupportedVersion) {
    return ResultCode::UnsupportedVersion;
  }
  if (a.result == ResultCode::CorruptFile ||
      b.result == ResultCode::CorruptFile) {
    return ResultCode::CorruptFile;
  }
  if (a.result != ResultCode::EmptyRecording) return a.result;
  return b.result;
}

bool operationalFailure(const Candidate& candidate) {
  return candidate.result != ResultCode::Ok &&
         candidate.result != ResultCode::EmptyRecording &&
         candidate.result != ResultCode::CorruptFile &&
         candidate.result != ResultCode::UnsupportedVersion &&
         candidate.result != ResultCode::WrongRobot;
}

}  // namespace

Storage::Storage(FileStore& files) : files_(files) {}

ResultCode Storage::load(std::uint8_t slot, RobotIdentity robot,
                         DecodedRecording& out) const {
  if (!validSlot(slot)) return ResultCode::InvalidSlot;
  char pathA[40]{};
  char pathB[40]{};
  pathFor(slot, 'a', pathA, sizeof(pathA));
  pathFor(slot, 'b', pathB, sizeof(pathB));
  const Candidate a =
      readCandidate(files_, pathA, robot, encodedA_, decodedA_);
  const Candidate b =
      readCandidate(files_, pathB, robot, encodedB_, decodedB_);
  const Generation selected =
      chooseGeneration({a.valid, a.generation}, {b.valid, b.generation});
  if (selected == Generation::A) {
    out = decodedA_;
    return ResultCode::Ok;
  }
  if (selected == Generation::B) {
    out = decodedB_;
    return ResultCode::Ok;
  }
  return failureFor(a, b);
}

SlotSummary Storage::inspect(std::uint8_t slot, RobotIdentity robot) const {
  SlotSummary summary{};
  const ResultCode result = load(slot, robot, decodedVerify_);
  summary.result = result;
  if (result != ResultCode::Ok) return summary;
  summary.valid = true;
  summary.generation = decodedVerify_.generation;
  summary.durationMs = decodedVerify_.capture.durationMs;
  summary.startX = decodedVerify_.route.start.x;
  summary.startY = decodedVerify_.route.start.y;
  summary.startHeading = decodedVerify_.route.start.heading;
  return summary;
}

ResultCode Storage::save(std::uint8_t slot, RobotIdentity robot,
                         const Capture& capture,
                         const ProcessedRoute& route) {
  if (!validSlot(slot)) return ResultCode::InvalidSlot;
  if (capture.robot != robot) return ResultCode::WrongRobot;
  char pathA[40]{};
  char pathB[40]{};
  pathFor(slot, 'a', pathA, sizeof(pathA));
  pathFor(slot, 'b', pathB, sizeof(pathB));
  const Candidate a =
      readCandidate(files_, pathA, robot, encodedA_, decodedA_);
  const Candidate b =
      readCandidate(files_, pathB, robot, encodedB_, decodedB_);
  if (operationalFailure(a)) return a.result;
  if (operationalFailure(b)) return b.result;

  std::uint32_t newest = 0;
  if (a.valid) newest = a.generation;
  if (b.valid && b.generation > newest) newest = b.generation;
  if (newest == std::numeric_limits<std::uint32_t>::max()) {
    return ResultCode::WriteFailed;
  }
  const std::uint32_t nextGeneration = newest + 1;
  const char* target = nullptr;
  if (!a.valid) {
    target = pathA;
  } else if (!b.valid) {
    target = pathB;
  } else {
    target = a.generation <= b.generation ? pathA : pathB;
  }

  const ResultCode encodeResult =
      encode(capture, route, nextGeneration, encodedWrite_);
  if (encodeResult != ResultCode::Ok) return encodeResult;
  const ResultCode writeResult =
      files_.write(target, encodedWrite_.data.data(), encodedWrite_.size);
  if (writeResult != ResultCode::Ok) return writeResult;

  const ResultCode verifyRead = files_.read(target, encodedA_);
  if (verifyRead != ResultCode::Ok) return verifyRead;
  const ResultCode verifyDecode = decode(encodedA_.data.data(), encodedA_.size,
                                         robot, decodedVerify_);
  if (verifyDecode != ResultCode::Ok) return verifyDecode;
  return decodedVerify_.generation == nextGeneration ? ResultCode::Ok
                                                      : ResultCode::CorruptFile;
}

ResultCode Storage::erase(std::uint8_t slot) {
  if (!validSlot(slot)) return ResultCode::InvalidSlot;
  char pathA[40]{};
  char pathB[40]{};
  pathFor(slot, 'a', pathA, sizeof(pathA));
  pathFor(slot, 'b', pathB, sizeof(pathB));
  const ResultCode resultA = files_.erase(pathA);
  const ResultCode resultB = files_.erase(pathB);
  if (resultA != ResultCode::Ok) return resultA;
  return resultB;
}

}  // namespace aon::shadow
