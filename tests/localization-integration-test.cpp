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
  CHECK(header.find("void resetPose(") != std::string::npos);
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
  CHECK(source.find("Odometry::Odometry(const Odometry&") ==
        std::string::npos);
  CHECK(count(combined, "pros::Mutex") == 1U);
  CHECK(source.find("encoderBack_.get_position()") != std::string::npos);

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

}  // namespace

int main() {
  odometryExposesOneCoherentInterface();
  drivetrainUsesTheHardwareOwnedEstimator();
  lemlibSharesTheHardwareOwnedTrackingDevices();
  std::cout << "localization integration tests passed\n";
  return 0;
}
