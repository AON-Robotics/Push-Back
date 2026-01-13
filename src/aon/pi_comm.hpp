#pragma once

 #include <cstdint>
 #include <functional>
 #include <string>
 #include <unordered_map>

 #include "okapi/api.hpp"

 namespace aon {

 // okapi velocity units (rpm)

/**
 * @brief Records the timestamp in milliseconds of the most recent heartbeat from the Pi.
 *
 * Exposed so other modules can inspect link status or plug in their own safety logic.
 */
extern std::uint32_t lastHeartbeatMs;

/**
 * @brief Initializes the USB serial channel and clears internal buffers.
 *
 * Call this once during startup before any other function in the module.
 */
void initUsbSerial();

/**
 * @brief Enqueues a message to be sent to the Raspberry Pi.
 *
 * @param line Text to transmit. The function ensures it ends with a newline.
 */
void sendToPi(const std::string &line);

/**
 * @brief Command handler map resolving names to their execution callbacks.
 *
 * Extend it by inserting new entries before initializing the communication flow.
 */
extern std::unordered_map<std::string, std::function<void(int)>> handlers;

/**
 * @brief Processes a complete textual line received from the Pi.
 *
 * @param msg Incoming string (without terminators). Supports CMD and heartbeat (HB).
 */
void handleRawCommand(const std::string &msg);

/**
 * @brief Watches how much time has elapsed since the last heartbeat.
 *
 * Useful to trigger an external kill-switch once the interval exceeds your threshold.
 */
void heartbeatWatchdog();

/**
 * @brief Polls the USB serial port.
 *
 * This function reads incoming commands, assembles full lines, and flushes queued
 * messages to the Pi while respecting buffer availability.
 */
void pollUsbSerial();

// Diagnostics for LCD/telemetry
std::string getLastReceivedMessage();
std::string getLastSentMessage();
std::size_t getOutgoingQueueSize();

}  // namespace aon
