#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

std::string readFile(const char* path) {
  std::ifstream input(path);
  CHECK(input.good());
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

std::size_t count(const std::string& text, const std::string& token) {
  std::size_t matches = 0;
  std::size_t position = 0;
  while ((position = text.find(token, position)) != std::string::npos) {
    ++matches;
    position += token.size();
  }
  return matches;
}

void odometryExposesOneCoherentInterface() {
  const std::string header =
      readFile("include/aon/odometry/odometry.hpp");
  const std::string source = readFile("src/aon/odometry.cpp");
  const std::string combined = header + source;

  CHECK(header.find("Pose getPose()") != std::string::npos);
  CHECK(header.find("Pose rawOdometryPose()") != std::string::npos);
  CHECK(header.find("bool resetPose(") != std::string::npos);
  CHECK(header.find("void update()") != std::string::npos);
  CHECK(header.find("LocalizationDiagnostics getDiagnostics()") !=
        std::string::npos);
  CHECK(header.find("bool calibrateImu(") != std::string::npos);

  CHECK(combined.find("changeWeb") == std::string::npos);
  CHECK(combined.find("GYRO_CONFIDENCE") == std::string::npos);
  CHECK(combined.find("pros::delay(3000)") == std::string::npos);
  CHECK(combined.find("pros::delay(2000)") == std::string::npos);
  CHECK(combined.find("gpsPosition()") == std::string::npos);
  CHECK(header.find("Odometry(const Odometry&) = delete") !=
        std::string::npos);
  CHECK(header.find("Odometry(Odometry&&) = delete") !=
        std::string::npos);
  CHECK(header.find("operator=(Odometry&&) = delete") !=
        std::string::npos);
  CHECK(source.find("Odometry::Odometry(const Odometry&") ==
        std::string::npos);
  CHECK(count(combined, "pros::Mutex") == 2U);
  CHECK(source.find("encoderBack_.get_position()") != std::string::npos);
  CHECK(header.find("localization::VelocityEstimator velocityEstimator_") !=
        std::string::npos);
  CHECK(source.find("candidateVelocityEstimator.update(") !=
        std::string::npos);
  const std::size_t getter = source.find("Pose Odometry::getPose()");
  CHECK(getter != std::string::npos);
  const std::size_t getterEnd = source.find("\n}\n", getter);
  CHECK(getterEnd != std::string::npos);
  const std::string getterBody = source.substr(getter, getterEnd - getter);
  CHECK(getterBody.find("getX()") == std::string::npos);
  CHECK(getterBody.find("getY()") == std::string::npos);
  CHECK(getterBody.find("getDegrees()") == std::string::npos);
}

void drivetrainUsesTheHardwareOwnedEstimator() {
  const std::string drivetrain =
      readFile("include/aon/drivetrain/drivetrain.hpp");
  const std::string hardware = readFile("src/aon/core/hardware.cpp");

  CHECK(drivetrain.find("Odometry& odometry") != std::string::npos);
  CHECK(drivetrain.find("std::unique_ptr<Odometry>") == std::string::npos);
  CHECK(drivetrain.find("Pose pose;") == std::string::npos);
  CHECK(hardware.find("std::make_unique<Odometry>") == std::string::npos);
}

void lemlibSharesTheHardwareOwnedTrackingDevices() {
  const std::string chassis = readFile("src/aon/lemlib/chassis.cpp");
  const std::string odometry =
      readFile("include/aon/odometry/odometry.hpp");

  CHECK(odometry.find("pros::Rotation encoderLeft_") != std::string::npos);
  CHECK(odometry.find("pros::Imu imu_") != std::string::npos);
  CHECK(chassis.find("static pros::Rotation sensor") == std::string::npos);
  CHECK(chassis.find("static pros::Imu sensor") == std::string::npos);
  CHECK(chassis.find("hardware().odometry.leftTrackingSensor()") !=
        std::string::npos);
  CHECK(chassis.find("hardware().odometry.imuSensor()") !=
        std::string::npos);
}

void localizationSchedulingIsDeterministicAndQuiet() {
  const std::string header =
      readFile("include/aon/odometry/odometry.hpp");
  const std::string source = readFile("src/aon/odometry.cpp");
  const std::string legacy =
      readFile("src/aon/drivetrain/legacy-motion.cpp");

  CHECK(header.find("void runLocalizationLoop()") != std::string::npos);
  CHECK(header.find("void initialize()") == std::string::npos);
  CHECK(source.find("pros::Task::delay_until") != std::string::npos);
  CHECK(legacy.find("runLocalizationLoop()") != std::string::npos);

  const std::size_t update = source.find("void Odometry::update()");
  const std::size_t updateEnd =
      source.find("bool Odometry::calibrateImu", update);
  CHECK(update != std::string::npos && updateEnd != std::string::npos);
  const std::string updateBody = source.substr(update, updateEnd - update);
  CHECK(updateBody.find("pros::delay") == std::string::npos);
  CHECK(updateBody.find("printf") == std::string::npos);
  CHECK(updateBody.find("cout") == std::string::npos);
  CHECK(updateBody.find("lcd::") == std::string::npos);

  const std::size_t snapshot =
      updateBody.find("const std::uint32_t generation");
  const std::size_t sensorRead = updateBody.find("encoderLeft_.get_position()");
  CHECK(snapshot != std::string::npos && sensorRead != std::string::npos);
  CHECK(snapshot < sensorRead);
  CHECK(source.find("if (!std::isfinite(x) || !std::isfinite(y) ||") !=
        std::string::npos);
  CHECK(source.find("consumeWheelDistances(currentWheels,") !=
        std::string::npos);
  CHECK(source.find("candidateBaselines = currentWheels") ==
        std::string::npos);
  CHECK(source.find("publisher(snapshotPose)") != std::string::npos);
  CHECK(header.find("pros::Mutex publicationMutex_") != std::string::npos);

  const std::size_t publishFunction =
      source.find("void Odometry::publishCurrent");
  const std::size_t publishCall =
      source.find("publisher(snapshotPose);", publishFunction);
  const std::size_t snapshotLock =
      source.find("TimedMutexLock snapshotLock", publishFunction);
  const std::size_t snapshotScopeEnd =
      source.find("\n  }\n", snapshotLock);
  CHECK(publishFunction != std::string::npos &&
        publishCall != std::string::npos);
  CHECK(snapshotLock != std::string::npos &&
        snapshotScopeEnd != std::string::npos);
  CHECK(snapshotScopeEnd < publishCall);
}

void fusedLemLibModeIsPresentButAuthorizationGated() {
  const std::string chassis = readFile("src/aon/lemlib/chassis.cpp");
  const std::string chassisHeader =
      readFile("include/aon/lemlib/chassis.hpp");
  const std::string actions = readFile("src/aon/auton/actions.cpp");
  const std::string legacy =
      readFile("src/aon/drivetrain/legacy-motion.cpp");

  CHECK(chassis.find("fusedLemLibAuthorized") != std::string::npos);
  CHECK(chassis.find("nullptr, nullptr, nullptr, nullptr, nullptr") !=
        std::string::npos);
  CHECK(chassis.find("calibrateImu()") != std::string::npos);
  CHECK(chassis.find("calibrate(false)") != std::string::npos);
  CHECK(chassis.find("runLocalizationLoop(publishFusedPose)") !=
        std::string::npos);
  CHECK(chassisHeader.find("TaskStartResult startFusedLocalization()") !=
        std::string::npos);
  CHECK(legacy.find("startFusedLocalization()") != std::string::npos);
  CHECK(chassis.find("get_state()") != std::string::npos);
  CHECK(legacy.find("get_state()") != std::string::npos);
  CHECK(count(chassis, "localizationBootReady.store(false);") >= 2U);
  CHECK(chassisHeader.find("bool localizationReady()") != std::string::npos);
  CHECK(chassis.find("localizationBootReady") != std::string::npos);
  CHECK(actions.find("MotionLease(const MotionLease&) = delete") !=
        std::string::npos);
  CHECK(actions.find("MotionLease(MotionLease&&) = delete") !=
        std::string::npos);
  CHECK(actions.find("~MotionLease() noexcept") != std::string::npos);
  CHECK(actions.find("if (!odometry.resetPose(x, y, heading))") !=
        std::string::npos);

  const std::size_t setPose = actions.find("void Actions::setPose");
  CHECK(setPose != std::string::npos);
  const std::size_t reset = actions.find("odometry.resetPose", setPose);
  const std::size_t publish = actions.find("chassis().setPose", setPose);
  CHECK(reset != std::string::npos);
  CHECK(publish != std::string::npos);
  CHECK(reset < publish);
}

}  // namespace

int main() {
  odometryExposesOneCoherentInterface();
  drivetrainUsesTheHardwareOwnedEstimator();
  lemlibSharesTheHardwareOwnedTrackingDevices();
  localizationSchedulingIsDeterministicAndQuiet();
  fusedLemLibModeIsPresentButAuthorizationGated();
  std::cout << "localization integration tests passed\n";
  return 0;
}
