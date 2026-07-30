#pragma once

#include "aon/shadow/types.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace aon::shadow {

constexpr std::size_t kSdWriteChunkBytes = 4096;

inline bool validSdPayload(const std::uint8_t* data, std::size_t size,
                           std::size_t maximumSize) {
  return data != nullptr && size != 0 && size <= maximumSize;
}

inline ResultCode cleanupFailureFor(ResultCode primary) {
  if (primary == ResultCode::WriteFailed) {
    return ResultCode::WriteCleanupFailed;
  }
  if (primary == ResultCode::FlushFailed) {
    return ResultCode::FlushCleanupFailed;
  }
  return ResultCode::CloseCleanupFailed;
}

template <typename Write, typename Flush, typename Close, typename Erase>
ResultCode writeSdPayload(const std::uint8_t* data, std::size_t size,
                          Write write, Flush flush, Close close, Erase erase) {
  ResultCode result = ResultCode::Ok;
  std::size_t offset = 0;
  while (offset < size) {
    const std::size_t chunk =
        std::min(kSdWriteChunkBytes, size - offset);
    if (write(data + offset, chunk) != chunk) {
      result = ResultCode::WriteFailed;
      break;
    }
    offset += chunk;
  }

  if (result == ResultCode::Ok && !flush()) {
    result = ResultCode::FlushFailed;
  }
  if (!close() && result == ResultCode::Ok) {
    result = ResultCode::CloseFailed;
  }
  if (result != ResultCode::Ok && !erase()) {
    return cleanupFailureFor(result);
  }
  return result;
}

}  // namespace aon::shadow
