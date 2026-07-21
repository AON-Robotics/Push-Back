#pragma once

#include "aon/shadow/codec.hpp"

#include <cstddef>
#include <cstdint>

namespace aon::shadow {

struct SlotSummary {
  ResultCode result = ResultCode::EmptyRecording;
  bool valid = false;
  std::uint32_t generation = 0;
  std::uint32_t durationMs = 0;
  float startX = 0;
  float startY = 0;
  float startHeading = 0;
};

class FileStore {
 public:
  virtual ~FileStore() = default;
  virtual ResultCode read(const char* path, EncodedRecording& out) const = 0;
  virtual ResultCode write(const char* path, const std::uint8_t* data,
                           std::size_t size) = 0;
  virtual ResultCode erase(const char* path) = 0;
};

class Storage {
 public:
  explicit Storage(FileStore& files);

  SlotSummary inspect(std::uint8_t slot, RobotIdentity robot) const;
  ResultCode load(std::uint8_t slot, RobotIdentity robot,
                  DecodedRecording& out) const;
  ResultCode save(std::uint8_t slot, RobotIdentity robot,
                  const Capture& capture, const ProcessedRoute& route);
  ResultCode erase(std::uint8_t slot);

 private:
  FileStore& files_;
  mutable EncodedRecording encodedA_{};
  mutable EncodedRecording encodedB_{};
  mutable EncodedRecording encodedWrite_{};
  mutable DecodedRecording decodedA_{};
  mutable DecodedRecording decodedB_{};
  mutable DecodedRecording decodedVerify_{};
};

class SdFileStore final : public FileStore {
 public:
  ResultCode read(const char* path, EncodedRecording& out) const override;
  ResultCode write(const char* path, const std::uint8_t* data,
                   std::size_t size) override;
  ResultCode erase(const char* path) override;
};

}  // namespace aon::shadow
