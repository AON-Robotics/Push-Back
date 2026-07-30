#include "aon/shadow/sd-write.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#define CHECK(value)                                                        \
  do {                                                                      \
    if (!(value)) {                                                         \
      std::cerr << #value << '\n';                                          \
      std::exit(1);                                                         \
    }                                                                       \
  } while (false)

struct FakeSink {
  std::vector<std::uint8_t> bytes;
  int writes = 0;
  int failWrite = 0;
  bool flushOk = true;
  bool closeOk = true;
  bool closed = false;
  bool erased = false;

  std::size_t write(const std::uint8_t* data, std::size_t size) {
    ++writes;
    if (size > aon::shadow::kSdWriteChunkBytes || writes == failWrite) {
      return 0;
    }
    bytes.insert(bytes.end(), data, data + size);
    return size;
  }
};

int main() {
  using aon::shadow::ResultCode;
  using aon::shadow::validSdPayload;
  using aon::shadow::writeSdPayload;

  std::array<std::uint8_t, 10000> payload{};
  for (std::size_t index = 0; index < payload.size(); ++index) {
    payload[index] = static_cast<std::uint8_t>(index);
  }
  CHECK(validSdPayload(payload.data(), payload.size(), payload.size()));
  CHECK(!validSdPayload(nullptr, payload.size(), payload.size()));
  CHECK(!validSdPayload(payload.data(), 0, payload.size()));
  CHECK(!validSdPayload(payload.data(), payload.size(), payload.size() - 1));

  FakeSink successful;
  CHECK(writeSdPayload(
            payload.data(), payload.size(),
            [&](const std::uint8_t* data, std::size_t size) {
              return successful.write(data, size);
            },
            [&] { return successful.flushOk; },
            [&] {
              successful.closed = true;
              return successful.closeOk;
            },
            [&] {
              successful.erased = true;
              return true;
            }) == ResultCode::Ok);
  CHECK(successful.writes == 3);
  CHECK(successful.bytes.size() == payload.size());
  CHECK(std::equal(payload.begin(), payload.end(), successful.bytes.begin(),
                   successful.bytes.end()));
  CHECK(successful.closed);
  CHECK(!successful.erased);

  FakeSink failedWrite;
  failedWrite.failWrite = 2;
  CHECK(writeSdPayload(
            payload.data(), payload.size(),
            [&](const std::uint8_t* data, std::size_t size) {
              return failedWrite.write(data, size);
            },
            [&] { return true; },
            [&] {
              failedWrite.closed = true;
              return true;
            },
            [&] {
              failedWrite.erased = true;
              return true;
            }) == ResultCode::WriteFailed);
  CHECK(failedWrite.closed);
  CHECK(failedWrite.erased);

  FakeSink failedFlush;
  failedFlush.flushOk = false;
  CHECK(writeSdPayload(
            payload.data(), payload.size(),
            [&](const std::uint8_t* data, std::size_t size) {
              return failedFlush.write(data, size);
            },
            [&] { return failedFlush.flushOk; },
            [&] {
              failedFlush.closed = true;
              return true;
            },
            [&] {
              failedFlush.erased = true;
              return true;
            }) == ResultCode::FlushFailed);
  CHECK(failedFlush.closed);
  CHECK(failedFlush.erased);

  FakeSink failedClose;
  failedClose.closeOk = false;
  CHECK(writeSdPayload(
            payload.data(), payload.size(),
            [&](const std::uint8_t* data, std::size_t size) {
              return failedClose.write(data, size);
            },
            [&] { return true; },
            [&] {
              failedClose.closed = true;
              return failedClose.closeOk;
            },
            [&] {
              failedClose.erased = true;
              return true;
            }) == ResultCode::CloseFailed);
  CHECK(failedClose.closed);
  CHECK(failedClose.erased);

  FakeSink failedCleanup;
  failedCleanup.failWrite = 1;
  CHECK(writeSdPayload(
            payload.data(), payload.size(),
            [&](const std::uint8_t* data, std::size_t size) {
              return failedCleanup.write(data, size);
            },
            [&] { return true; },
            [&] { return true; },
            [&] {
              failedCleanup.erased = true;
              return false;
            }) == ResultCode::WriteCleanupFailed);
  CHECK(failedCleanup.erased);

  std::cout << "shadow SD write tests passed\n";
}
