#include "aon/core/hardware.hpp"

#include <memory>

namespace aon::core {

Hardware::Hardware()
#if USING_BIG_ROBOT
    : scaler(SENSITIVITY),
      driver(operator_control::FABIAN),
      startingPose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y,
                   INITIAL_ODOMETRY_THETA),
      odometry(5, -6, 7, 0, 14),
      speedFactors(0.6, 1.0, 0.6, 1.0, 1.0, 1.0),
      xProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
      yProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
      thetaProfile(MAX_RPM, MAX_ACCEL * 3, MAX_DECEL * 0.8, MAX_ACCEL * 3),
      drivetrain({12, -13, -18, 19}, {-1, 2, 3, -4}, {-15}, startingPose,
                 std::make_unique<Odometry>(odometry), speedFactors,
                 std::make_unique<MotionProfile>(xProfile),
                 std::make_unique<MotionProfile>(yProfile),
                 std::make_unique<MotionProfile>(thetaProfile)),
      intake({20, -11, -10}, {17}, 'H', 9, 16, 'F', 'E'),
      sem('G', Piston::RETRACTED),
      brooks('D', Piston::RETRACTED),
#else
    : scaler(SENSITIVITY),
      driver(operator_control::KEVIN),
      startingPose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y,
                   INITIAL_ODOMETRY_THETA),
      odometry(19, -18, 5, 0, 16),
      speedFactors(0.6, 0.0, 0.6, 1.0, 0.0, 0.667),
      yProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
      thetaProfile(MAX_RPM, MAX_ACCEL * 3, MAX_DECEL * 0.8, MAX_ACCEL * 3),
      drivetrain({11, -12, 13, -14}, {1, -2, 3, -4}, startingPose,
                 std::make_unique<Odometry>(odometry), speedFactors,
                 std::make_unique<MotionProfile>(yProfile),
                 std::make_unique<MotionProfile>(thetaProfile)),
      intake({-9}, {-6}, {7}, {-8}, 'H', 'B', 'A', 20, 17),
      arrow('C', Piston::RETRACTED),
      brooks('G', Piston::RETRACTED),
#endif
      orbit(0, true, 0, 0),
      alliance(Alliance::Red),
      potentiometer('Z'),
      drivePID(0.02, 0, 0),
      turnPID(0.002, 0, 0),
      fastPID(1, 0, 0),
      mainController(pros::E_CONTROLLER_MASTER) {}

Hardware& hardware() {
  static Hardware instance;
  return instance;
}

}  // namespace aon::core
