#include "aon/shadow/storage.hpp"

#include "pros/misc.hpp"

#include <cerrno>
#include <cstdio>

namespace aon::shadow {
namespace {

ResultCode openFailure() {
  if (errno == EROFS) return ResultCode::ReadOnly;
  return errno == ENOENT ? ResultCode::EmptyRecording
                         : ResultCode::OpenFailed;
}

}  // namespace

ResultCode SdFileStore::read(const char* path, EncodedRecording& out) const {
  out.size = 0;
  if (!pros::usd::is_installed()) return ResultCode::NoSd;
  std::FILE* file = std::fopen(path, "rb");
  if (file == nullptr) return openFailure();

  ResultCode result = ResultCode::Ok;
  if (std::fseek(file, 0, SEEK_END) != 0) {
    result = ResultCode::ReadFailed;
  } else {
    const long length = std::ftell(file);
    if (length < 0 ||
        static_cast<unsigned long>(length) > kMaximumEncodedBytes ||
        std::fseek(file, 0, SEEK_SET) != 0) {
      result = ResultCode::ReadFailed;
    } else {
      out.size = static_cast<std::size_t>(length);
      if (out.size != 0 &&
          std::fread(out.data.data(), 1, out.size, file) != out.size) {
        out.size = 0;
        result = ResultCode::ReadFailed;
      }
    }
  }
  if (std::fclose(file) != 0 && result == ResultCode::Ok) {
    result = ResultCode::CloseFailed;
  }
  return result;
}

ResultCode SdFileStore::write(const char* path, const std::uint8_t* data,
                              std::size_t size) {
  if (!pros::usd::is_installed()) return ResultCode::NoSd;
  if (data == nullptr || size > kMaximumEncodedBytes) {
    return ResultCode::WriteFailed;
  }
  std::FILE* file = std::fopen(path, "wb");
  if (file == nullptr) return openFailure();

  const bool wrote = std::fwrite(data, 1, size, file) == size;
  const bool flushed = std::fflush(file) == 0;
  const bool closed = std::fclose(file) == 0;
  if (!wrote) return ResultCode::WriteFailed;
  if (!flushed) return ResultCode::FlushFailed;
  if (!closed) return ResultCode::CloseFailed;
  return ResultCode::Ok;
}

ResultCode SdFileStore::erase(const char* path) {
  if (!pros::usd::is_installed()) return ResultCode::NoSd;
  if (std::remove(path) == 0 || errno == ENOENT) return ResultCode::Ok;
  return ResultCode::DeleteFailed;
}

}  // namespace aon::shadow
