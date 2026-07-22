#include "aon/shadow/sd-directory.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>

#define CHECK(value)                                                        \
  do {                                                                      \
    if (!(value)) {                                                         \
      std::cerr << #value << '\n';                                          \
      std::exit(1);                                                         \
    }                                                                       \
  } while (false)

int main() {
  using aon::shadow::inspectRootListing;
  using aon::shadow::beginSdReadOpenFailure;
  using aon::shadow::finishSdReadOpenFailure;
  using aon::shadow::RootListingPathStatus;
  using aon::shadow::SdReadOpenDecision;

  char empty[2]{};
  CHECK(inspectRootListing(empty, sizeof(empty),
                           "/usd/aon-shadow-slot-1-a.bin") ==
        RootListingPathStatus::Absent);
  char oneFile[64] = "aon-shadow-slot-1-a.bin\n";
  CHECK(inspectRootListing(oneFile, sizeof(oneFile),
                           "/usd/aon-shadow-slot-1-a.bin") ==
        RootListingPathStatus::Present);
  char severalFiles[128] =
      "notes.txt\r\naon-shadow-slot-2-b.bin\r\nother.bin\r\n";
  CHECK(inspectRootListing(severalFiles, sizeof(severalFiles),
                           "/usd/aon-shadow-slot-2-b.bin") ==
        RootListingPathStatus::Present);
  char longerName[64] = "aon-shadow-slot-1-a.bin.bak\n";
  CHECK(inspectRootListing(longerName, sizeof(longerName),
                           "/usd/aon-shadow-slot-1-a.bin") ==
        RootListingPathStatus::Absent);
  char shorterName[64] = "aon-shadow-slot-1-a.bi\n";
  CHECK(inspectRootListing(shorterName, sizeof(shorterName),
                           "/usd/aon-shadow-slot-1-a.bin") ==
        RootListingPathStatus::Absent);
  CHECK(inspectRootListing(oneFile, sizeof(oneFile),
                           "/usd/sub/aon-shadow-slot-1-a.bin") ==
        RootListingPathStatus::Incomplete);
  CHECK(inspectRootListing(nullptr, 0, "/usd/a.bin") ==
        RootListingPathStatus::Incomplete);
  CHECK(inspectRootListing(oneFile, sizeof(oneFile), nullptr) ==
        RootListingPathStatus::Incomplete);
  const char truncated[] = {'a', '.', 'b', 'i', 'n'};
  CHECK(inspectRootListing(truncated, sizeof(truncated), "/usd/a.bin") ==
        RootListingPathStatus::Incomplete);
  char nulTerminatedTruncation[6] = {'a', '.', 'b', 'i', 'n', '\0'};
  CHECK(inspectRootListing(nulTerminatedTruncation,
                           sizeof(nulTerminatedTruncation), "/usd/a.bin") ==
        RootListingPathStatus::Incomplete);
  char boundedWholeEntries[1024]{};
  std::memset(boundedWholeEntries, 'x', 600);
  boundedWholeEntries[600] = '\n';
  boundedWholeEntries[601] = '\0';
  CHECK(inspectRootListing(boundedWholeEntries, sizeof(boundedWholeEntries),
                           "/usd/a.bin", 512) ==
        RootListingPathStatus::Incomplete);

  CHECK(beginSdReadOpenFailure(ENFILE) ==
        SdReadOpenDecision::NeedsDirectory);
  CHECK(beginSdReadOpenFailure(ENOENT) ==
        SdReadOpenDecision::NeedsDirectory);
  CHECK(beginSdReadOpenFailure(EROFS) == SdReadOpenDecision::ReadOnly);
  CHECK(beginSdReadOpenFailure(EIO) == SdReadOpenDecision::OpenFailed);
  CHECK(finishSdReadOpenFailure(true, 0, RootListingPathStatus::Absent) ==
        SdReadOpenDecision::EmptyRecording);
  CHECK(finishSdReadOpenFailure(true, 0, RootListingPathStatus::Present) ==
        SdReadOpenDecision::OpenFailed);
  CHECK(finishSdReadOpenFailure(true, 0, RootListingPathStatus::Incomplete) ==
        SdReadOpenDecision::OpenFailed);
  CHECK(finishSdReadOpenFailure(false, ENOENT,
                                RootListingPathStatus::Absent) ==
        SdReadOpenDecision::OpenFailed);
  CHECK(finishSdReadOpenFailure(false, EROFS,
                                RootListingPathStatus::Absent) ==
        SdReadOpenDecision::ReadOnly);

  std::cout << "shadow SD directory tests passed\n";
}
