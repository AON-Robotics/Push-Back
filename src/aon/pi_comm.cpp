#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include "okapi/api.hpp"
#include <queue>
#include <algorithm> 
#include <cstdint>    
#include <unistd.h>   
#include "pros/apix.h"
#include "pi_comm.hpp"
#include <cstdio>


namespace aon {
  std::uint32_t lastHeartbeatMs = 0;
  std::string lastReceivedMessage;
  std::string lastSentMessage;
// RP5 READOUT
//-----------------------------------------------------------------------------------------------------------
  static std::string usbBuffer;
  static std::queue<std::string> outgoingMessages;
  
  void initUsbSerial() {
    aon::lastHeartbeatMs = pros::millis();
    usbBuffer.reserve(128);
  }
  
  // Call this from anywhere in the codebase to send something to the Pi
  void sendToPi(const std::string& line) {
    // Ensure the message ends with '\n'
    if (!line.empty() && line.back() == '\n') {
      outgoingMessages.push(line);
    } else {
      outgoingMessages.push(line + '\n');
    }
  }

  
  // ---------------------------------------------------------------------------------------------
  

  
void handleDoubleTest(double value) {
  aon::sendToPi("ACK Double," + std::to_string(value));
}

void handleFloatTest(float value) {
  aon::sendToPi("ACK Float," + std::to_string(value));
}

void handleBoolTest(bool value) {
  aon::sendToPi("ACK Bool," + std::to_string(value));
}

std::unordered_map<std::string, std::function<void(int)>> handlers{
//   {"Move", [](int v) { handleMoveCommand(v); }},//tankdrive move
//   {"Turn", [](int v) { handleTurnCommand(v); }},//tankdrive turn
  {"test", [](int v) { handleDoubleTest(static_cast<double>(v)); }},
  {"test2", [](int v) { handleFloatTest(static_cast<float>(v)); }},
  {"test3", [](int v) { handleBoolTest(v != 0); }},
};

// Expects strings such as: "CMD Move,5" or "CMD Turn,-90"
void handleRawCommand(const std::string& msg) {
  lastReceivedMessage = msg;
  if (msg == "HB") {
    aon::lastHeartbeatMs = pros::millis();
    aon::sendToPi("ACK HB," + std::to_string(aon::lastHeartbeatMs));
    return;
  }
  
  if (msg.rfind("CMD ", 0) != 0) {
    return; // doesn't start with "CMD "
  }
  
  std::string rest = msg.substr(4); // after "CMD "
  std::size_t comma = rest.find(',');
  if (comma == std::string::npos) {
    return;
  }
  
  std::string name = rest.substr(0, comma); // "Move"
  std::string valueStr = rest.substr(comma + 1); // "5"
  
  int value = 0;
  try {
    value = std::stoi(valueStr);
  } catch (...) {
    std::cout << "[pi_comm] invalid value: " << valueStr << '\n';
    return;
  }
  
  auto it = aon::handlers.find(name);
  if (it != aon::handlers.end()){
    it->second(value); // calls handleMoveCommand(value), handleTurnCommand(value)...
  }
}

void heartbeatWatchdog(){
  const std::uint32_t now = pros::millis();
  const std::uint32_t dt  = now - aon::lastHeartbeatMs;
  // If more than 500 ms go by without heartbeat -> kill-switch
  // maybe flag linkLost = true
}

namespace {

void flushOutgoingMessages() {
  while (!outgoingMessages.empty()) {
    std::string nextMessage = outgoingMessages.front();
    outgoingMessages.pop();
    std::printf("%s", nextMessage.c_str());
    // Keep diagnostics readable by stripping the newline we add for transport.
    if (!nextMessage.empty() && nextMessage.back() == '\n') {
      nextMessage.pop_back();
    }
    lastSentMessage = nextMessage;
  }
}

}  // namespace

void pollUsbSerial() {
  constexpr std::size_t kBufferSize = 128;
  static char lineBuffer[kBufferSize];
  static std::size_t bufferIndex = 0;

  flushOutgoingMessages();

  const int incomingChar = std::getchar();
  if (incomingChar == EOF) {
    return;
  }
  if (incomingChar == '\r') {
    return;
  }
  if (incomingChar == '\n') {
    lineBuffer[bufferIndex] = '\0';
    handleRawCommand(lineBuffer);
    bufferIndex = 0;
    flushOutgoingMessages();
    return;
  }

  if (bufferIndex < kBufferSize - 1) {
    lineBuffer[bufferIndex++] = static_cast<char>(incomingChar);
  } else {
    bufferIndex = 0;
    sendToPi("VS_ERR:LONG");
    flushOutgoingMessages();
  }
}

std::string getLastReceivedMessage() {
  return lastReceivedMessage;
}

std::string getLastSentMessage() {
  return lastSentMessage;
}

std::size_t getOutgoingQueueSize() {
  return outgoingMessages.size();
}
} 
