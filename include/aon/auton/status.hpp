#pragma once

#include <cstdint>

namespace aon::auton {

/**
 * @brief Execution states shared by competition autonomous and the GUI.
 */
enum class RoutineState {
  Idle,
  Selected,
  Running,
  Completed,
  Failed,
};

/**
 * @brief Snapshot of the currently selected or executing autonomous routine.
 */
struct RoutineStatus {
  const char* name;
  RoutineState state;
  std::uint32_t startedAt;
  std::uint32_t finishedAt;
};

/**
 * @brief Records the routine selected by the GUI.
 * @param name Stable display name owned by the autonomous option table.
 */
void selectRoutine(const char* name);

/**
 * @brief Marks an autonomous routine as running.
 * @param name Stable display name for the executing routine.
 */
void startRoutine(const char* name);

/**
 * @brief Marks the current autonomous routine as finished.
 * @param succeeded Whether the routine returned a successful result.
 */
void finishRoutine(bool succeeded);

/**
 * @brief Returns a thread-safe copy of the current autonomous state.
 * @return Current routine status snapshot.
 */
RoutineStatus routineStatus();

}  // namespace aon::auton
