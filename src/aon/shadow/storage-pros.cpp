#include "aon/shadow/storage.hpp"

#include "aon/shadow/sd-directory.hpp"

#include "pros/error.h"
#include "pros/misc.hpp"

#include <cerrno>
#include <cstring>
#include <cstdio>

namespace aon::shadow {
namespace {

ResultCode resultFor(SdReadOpenDecision decision) {
  if (decision == SdReadOpenDecision::EmptyRecording) {
    return ResultCode::EmptyRecording;
  }
  if (decision == SdReadOpenDecision::ReadOnly) return ResultCode::ReadOnly;
  return ResultCode::OpenFailed;
}

ResultCode classifyMissingRead(const char* path, int openError) {
  const SdReadOpenDecision initial = beginSdReadOpenFailure(openError);
  if (initial != SdReadOpenDecision::NeedsDirectory) {
    return resultFor(initial);
  }
  // PROS 4.2.2 maps every null vexFileOpen() result to ENFILE, including a
  // normal missing file. Ask the directory API, which preserves filesystem
  // errors, before deciding whether an empty slot is an operational failure.
  // Keep this off the task stack and leave enough unused tail space for any
  // FAT long-file-name entry. A listing close to the bound is ambiguous and
  // fails closed instead of treating an unseen generation as absent.
  constexpr std::size_t kRootListingBytes = 64 * 1024;
  constexpr std::size_t kCompletenessMargin = 512;
  static char rootListing[kRootListingBytes];
  std::memset(rootListing, 0, sizeof(rootListing));
  const bool listingSucceeded =
      pros::usd::list_files("/", rootListing,
                            static_cast<std::int32_t>(sizeof(rootListing))) !=
      PROS_ERR;
  const int listingError = errno;
  const RootListingPathStatus status =
      inspectRootListing(rootListing, sizeof(rootListing), path,
                         kCompletenessMargin);
  return resultFor(
      finishSdReadOpenFailure(listingSucceeded, listingError, status));
}

ResultCode writeOpenFailure() {
  return errno == EROFS ? ResultCode::ReadOnly : ResultCode::OpenFailed;
}

}  // namespace

ResultCode SdFileStore::read(const char* path, EncodedRecording& out) const {
  out.size = 0;
  if (!pros::usd::is_installed()) return ResultCode::NoSd;
  std::FILE* file = std::fopen(path, "rb");
  if (file == nullptr) {
    const int openError = errno;
    return classifyMissingRead(path, openError);
  }

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
  if (file == nullptr) return writeOpenFailure();

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
