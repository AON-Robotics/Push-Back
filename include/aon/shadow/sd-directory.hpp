#pragma once

#include <cerrno>
#include <cstddef>
#include <cstring>

namespace aon::shadow {

enum class RootListingPathStatus { Absent, Present, Incomplete };
enum class SdReadOpenDecision {
  NeedsDirectory,
  EmptyRecording,
  ReadOnly,
  OpenFailed
};

inline SdReadOpenDecision beginSdReadOpenFailure(int openError) {
  if (openError == EROFS) return SdReadOpenDecision::ReadOnly;
  if (openError == ENFILE || openError == ENOENT) {
    return SdReadOpenDecision::NeedsDirectory;
  }
  return SdReadOpenDecision::OpenFailed;
}

inline SdReadOpenDecision finishSdReadOpenFailure(
    bool listingSucceeded, int listingError,
    RootListingPathStatus pathStatus) {
  if (!listingSucceeded) {
    return listingError == EROFS ? SdReadOpenDecision::ReadOnly
                                 : SdReadOpenDecision::OpenFailed;
  }
  return pathStatus == RootListingPathStatus::Absent
             ? SdReadOpenDecision::EmptyRecording
             : SdReadOpenDecision::OpenFailed;
}

inline RootListingPathStatus inspectRootListing(const char* listing,
                                                std::size_t capacity,
                                                const char* path,
                                                std::size_t minimumUnused = 0) {
  if (listing == nullptr || path == nullptr || capacity == 0) {
    return RootListingPathStatus::Incomplete;
  }
  const auto* terminator = static_cast<const char*>(
      std::memchr(listing, '\0', capacity));
  if (terminator == nullptr || terminator == listing + capacity - 1 ||
      static_cast<std::size_t>(listing + capacity - terminator - 1) <
          minimumUnused) {
    return RootListingPathStatus::Incomplete;
  }

  constexpr const char* prefix = "/usd/";
  constexpr std::size_t prefixLength = 5;
  if (std::strncmp(path, prefix, prefixLength) != 0) {
    return RootListingPathStatus::Incomplete;
  }
  const char* fileName = path + prefixLength;
  if (*fileName == '\0' || std::strchr(fileName, '/') != nullptr) {
    return RootListingPathStatus::Incomplete;
  }
  const std::size_t fileNameLength = std::strlen(fileName);

  const char* entry = listing;
  while (*entry != '\0') {
    const char* lineEnd = entry;
    while (*lineEnd != '\0' && *lineEnd != '\n' && *lineEnd != '\r') {
      ++lineEnd;
    }
    if (static_cast<std::size_t>(lineEnd - entry) == fileNameLength &&
        std::strncmp(entry, fileName, fileNameLength) == 0) {
      return RootListingPathStatus::Present;
    }
    while (*lineEnd == '\n' || *lineEnd == '\r') ++lineEnd;
    entry = lineEnd;
  }
  return RootListingPathStatus::Absent;
}

}  // namespace aon::shadow
