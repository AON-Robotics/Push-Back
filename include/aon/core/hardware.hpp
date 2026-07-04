#pragma once

#include "aon/constants.hpp"
#include "aon/control/driver.hpp"
#include "aon/controls/pid/pid.hpp"
#include "aon/controls/trapezoid-profile/trapezoid.hpp"
#include "aon/drivetrain/differential-drive.hpp"
#include "aon/drivetrain/h-drive.hpp"
#include "aon/intake/intake.hpp"
#include "aon/math/scaling/pilons-scaler.hpp"
#include "aon/odometry/odometry.hpp"
#include "aon/orbit/orbit.hpp"
#include "aon/piston/piston.hpp"

#include "pros/adi.hpp"
#include "pros/misc.hpp"

namespace aon::core {

/// Owns the legacy robot devices during the structural migration.
///
/// Member order intentionally matches the former globals.hpp construction
/// order. Public members provide a temporary compatibility surface while
/// control, autonomous, and GUI code are migrated to explicit dependencies.
class Hardware {
 public:
  Hardware();

  PilonsScaler scaler;
  operator_control::Driver driver;
  Pose startingPose;
  Odometry odometry;
  Drivetrain::SpeedFactors speedFactors;
#if USING_BIG_ROBOT
  MotionProfile xProfile;
#endif
  MotionProfile yProfile;
  MotionProfile thetaProfile;
#if USING_BIG_ROBOT
  HDrive drivetrain;
#else
  DifferentialDrive drivetrain;
#endif
  Intake intake;
#if USING_BIG_ROBOT
  Piston sem;
#else
  Piston arrow;
#endif
  Piston brooks;
  Orbit orbit;
  volatile Alliance alliance;
  pros::adi::Potentiometer potentiometer;
  PID drivePID;
  PID turnPID;
  PID fastPID;
  pros::Controller mainController;
};

Hardware& hardware();

}  // namespace aon::core
