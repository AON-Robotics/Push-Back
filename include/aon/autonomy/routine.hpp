#pragma once

#ifndef AON_AUTONOMY_ROUTINE_HPP_
#define AON_AUTONOMY_ROUTINE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "aon/autonomy/goto_pose.hpp"

namespace aon::autonomy {

enum class StepKind : std::uint8_t {
  go_to_pose = 0,
  wait = 1,
  action = 2,
};

struct GoToPoseStep {
  double x_in = 0.0;
  double y_in = 0.0;
  double heading_rad = 0.0;
  std::uint32_t timeout_ms = 8000;
  std::function<void()> concurrent_fn{};
};

struct Step {
  StepKind kind = StepKind::go_to_pose;
  GoToPoseStep go_to{};
  std::function<void()> action_fn{};
};

struct RoutineResult {
  std::size_t total_steps = 0;
  std::size_t completed_steps = 0;
  GoToPoseStatus last_status = GoToPoseStatus::reached;
};

class Routine {
 public:
  Routine &goTo(double x_in,
                double y_in,
                double heading_rad,
                std::uint32_t timeout_ms = 8000,
                std::function<void()> concurrent_fn = {});

  Routine &goToDeg(double x_in,
                   double y_in,
                   double heading_deg,
                   std::uint32_t timeout_ms = 8000,
                   std::function<void()> concurrent_fn = {});

  Routine &doAction(std::function<void()> fn);

  RoutineResult run() const;

 private:
  GoToPoseStatus runStep(const Step &step) const;

 private:
  std::vector<Step> steps_{};
};

}  // namespace aon::autonomy

#endif  // AON_AUTONOMY_ROUTINE_HPP_
