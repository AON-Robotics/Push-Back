#include "aon/core/hardware.hpp"

#include <memory>

#include "aon/config/hardware-map.hpp"

namespace aon::core {
namespace {

#if USING_BIG_ROBOT
constexpr const aon::config::RobotHardwareMap& selectedHardwareMap =
    aon::config::bigRobotHardwareMap;
#else
constexpr const aon::config::RobotHardwareMap& selectedHardwareMap =
    aon::config::smallRobotHardwareMap;
#endif

}  // namespace

Hardware::Hardware()
#if USING_BIG_ROBOT
    : scaler(SENSITIVITY),
      driver(operator_control::FABIAN),
      startingPose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y,
                   INITIAL_ODOMETRY_THETA),
      odometry(selectedHardwareMap.legacyTracking.left,
               selectedHardwareMap.legacyTracking.right,
               selectedHardwareMap.legacyTracking.back, 0,
               selectedHardwareMap.legacyTracking.imu),
      speedFactors(0.6, 1.0, 0.6, 1.0, 1.0, 1.0),
      xProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
      yProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
      thetaProfile(MAX_RPM, MAX_ACCEL * 3, MAX_DECEL * 0.8, MAX_ACCEL * 3),
      drivetrain({selectedHardwareMap.drive.left[0],
                  selectedHardwareMap.drive.left[1],
                  selectedHardwareMap.drive.left[2],
                  selectedHardwareMap.drive.left[3]},
                 {selectedHardwareMap.drive.right[0],
                  selectedHardwareMap.drive.right[1],
                  selectedHardwareMap.drive.right[2],
                  selectedHardwareMap.drive.right[3]},
                 {-15}, startingPose,
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
      odometry(selectedHardwareMap.legacyTracking.left,
               selectedHardwareMap.legacyTracking.right,
               selectedHardwareMap.legacyTracking.back, 0,
               selectedHardwareMap.legacyTracking.imu),
      speedFactors(0.6, 0.0, 0.6, 1.0, 0.0, 0.667),
      yProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
      thetaProfile(MAX_RPM, MAX_ACCEL * 3, MAX_DECEL * 0.8, MAX_ACCEL * 3),
      drivetrain({selectedHardwareMap.drive.left[0],
                  selectedHardwareMap.drive.left[1],
                  selectedHardwareMap.drive.left[2],
                  selectedHardwareMap.drive.left[3]},
                 {selectedHardwareMap.drive.right[0],
                  selectedHardwareMap.drive.right[1],
                  selectedHardwareMap.drive.right[2],
                  selectedHardwareMap.drive.right[3]},
                 startingPose,
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
