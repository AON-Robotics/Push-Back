#pragma once
#include <string>

// Simple command container.
// Missing fields keep defaults.
struct Command {
  float Move = 0.0;
  float Turn = 0.0;
};

// Parse lines like:
//   Move=10;Turn=-3;CameraServo=90\n
// Notes:
// - order doesn't matter
// - unknown keys are ignored
// - missing keys keep defaults
Command parseCommandKV(std::string line);
