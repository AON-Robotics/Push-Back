#include "aon/autonomy/routine.hpp"

#include "aon/tank-drive/tank-drive.hpp"

#include <iostream>
#include <utility>

extern aon::TankDrive drivetrainTank2;

namespace aon::autonomy {
namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(const double deg) {
  return deg * (kPi / 180.0);
}

const char *statusToString(const GoToPoseStatus status) {
  switch (status) {
    case GoToPoseStatus::reached:
      return "reached";
    case GoToPoseStatus::timed_out:
      return "timed_out";
    case GoToPoseStatus::no_path:
    default:
      return "no_path";
  }
}

}  // namespace

Routine &Routine::goTo(double x_in,
                       double y_in,
                       double heading_rad,
                       std::uint32_t timeout_ms,
                       std::function<void()> concurrent_fn) {
  Step step{};
  step.kind = StepKind::go_to_pose;
  step.go_to.x_in = x_in;
  step.go_to.y_in = y_in;
  step.go_to.heading_rad = heading_rad;
  step.go_to.timeout_ms = timeout_ms;
  step.go_to.concurrent_fn = std::move(concurrent_fn);
  steps_.push_back(step);
  return *this;
}

Routine &Routine::goToDeg(double x_in,
                          double y_in,
                          double heading_deg,
                          std::uint32_t timeout_ms,
                          std::function<void()> concurrent_fn) {
  return goTo(x_in, y_in, degToRad(heading_deg), timeout_ms, std::move(concurrent_fn));
}

Routine &Routine::doAction(std::function<void()> fn) {
  Step step{};
  step.kind = StepKind::action;
  step.action_fn = std::move(fn);
  steps_.push_back(std::move(step));
  return *this;
}

GoToPoseStatus Routine::runStep(const Step &step) const {
  switch (step.kind) {
    case StepKind::go_to_pose: {
      GoToPoseOptions opts{};
      opts.timeout_ms = step.go_to.timeout_ms;
      return goToPose(step.go_to.x_in,
                      step.go_to.y_in,
                      step.go_to.heading_rad,
                      opts);
    }
    case StepKind::wait:
    default:
      return GoToPoseStatus::no_path;
  }
}

RoutineResult Routine::run() const {
  RoutineResult out{};
  out.total_steps = steps_.size();
  out.last_status = GoToPoseStatus::reached;

  std::cout << "ROUTINE_BEGIN,total_steps," << out.total_steps << '\n';

  for (std::size_t idx = 0; idx < steps_.size(); ++idx) {
    const Step &step = steps_[idx];
    GoToPoseStatus status = GoToPoseStatus::reached;

    switch (step.kind) {
      case StepKind::go_to_pose: {
        std::cout << "ROUTINE_STEP_BEGIN"
                  << ",idx," << idx
                  << ",kind,go_to_pose"
                  << ",x," << step.go_to.x_in
                  << ",y," << step.go_to.y_in
                  << ",heading_rad," << step.go_to.heading_rad
                  << ",timeout_ms," << step.go_to.timeout_ms
                  << '\n';

        if (step.go_to.concurrent_fn) {
          pros::Task([fn = step.go_to.concurrent_fn] { fn(); });
        }
        status = runStep(step);
        ::drivetrainTank2.stop();
        break;
      }
      case StepKind::action: {
        const bool action_valid = static_cast<bool>(step.action_fn);
        std::cout << "ROUTINE_STEP_BEGIN"
                  << ",idx," << idx
                  << ",kind,action"
                  << ",action_valid," << static_cast<int>(action_valid)
                  << '\n';
        if (action_valid) {
          step.action_fn();
        }
        status = GoToPoseStatus::reached;
        break;
      }
      case StepKind::wait:
      default: {
        std::cout << "ROUTINE_STEP_BEGIN"
                  << ",idx," << idx
                  << ",kind,unsupported"
                  << '\n';
        status = GoToPoseStatus::no_path;
        break;
      }
    }

    out.last_status = status;

    std::cout << "ROUTINE_STEP_END"
              << ",idx," << idx
              << ",status," << statusToString(status)
              << ",completed_steps," << out.completed_steps
              << '\n';

    if (status == GoToPoseStatus::no_path) {
      std::cout << "ROUTINE_ABORT"
                << ",idx," << idx
                << ",status," << statusToString(status)
                << '\n';
      break;
    }

    out.completed_steps++;
  }

  std::cout << "ROUTINE_END"
            << ",completed_steps," << out.completed_steps
            << ",total_steps," << out.total_steps
            << ",last_status," << statusToString(out.last_status)
            << '\n';

  return out;
}

}  // namespace aon::autonomy
