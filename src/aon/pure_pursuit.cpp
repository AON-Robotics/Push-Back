#include "pure_pursuit.hpp"
// #include "aon/tank-drive/tank-drive.hpp"   // Include TankDrive
#include "globals.hpp"
#include "pros/misc.h"                     // For pros::delay
#include "pros/rtos.hpp"                   // For PROS timing
// #include "/Users/geraldrodriguez/Development/High-Stakes/Push-Back/include/aon/globals.hpp"
using namespace aon;
namespace aon {

template <typename T>
T clampValue(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

//  HELPER FUNCTIONS
double distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

// Overload to handle Robot–Point distance
double distance(const Robot& r, const Point& p) {
    return std::sqrt(std::pow(p.x - r.x, 2) + std::pow(p.y - r.y, 2));
}

double angleToPoint(const Robot& robot, const Point& target) {
    return std::atan2(target.y - robot.y, target.x - robot.x);
}

//  ADAPTIVE LOOKAHEAD FUNCTION
// here we will see how our adaptive look ahead function works
double computeAdaptiveLookahead(double speed) {
    double L = MIN_LOOKAHEAD + SPEED_SCALE * speed;
    return clampValue(L, MIN_LOOKAHEAD, MAX_LOOKAHEAD);
}

//  FIND LOOKAHEAD POINT ON PATH
Point findLookaheadPoint(const Robot& robot, const std::vector<Point>& path, double lookahead) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double d1 = distance(robot, path[i]);
        double d2 = distance(robot, path[i + 1]);
        if (d1 < lookahead && d2 >= lookahead) {
            double t = (lookahead - d1) / (d2 - d1);
            Point result;
            result.x = path[i].x + t * (path[i + 1].x - path[i].x);
            result.y = path[i].y + t * (path[i + 1].y - path[i].y);
            return result;
        }
    }
    return path.back();
}

//  UPDATE ROBOT STATE (for internal logic, not motion)
void updateRobot(Robot& robot, const Point& target) {
    double targetAngle = angleToPoint(robot, target);

    double angleDiff = targetAngle - robot.heading;
    while (angleDiff > M_PI)  angleDiff -= 2 * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2 * M_PI;

    // Limit turning rate for smoothness
    if (angleDiff > MAX_TURN_RATE)  angleDiff = MAX_TURN_RATE;
    if (angleDiff < -MAX_TURN_RATE) angleDiff = -MAX_TURN_RATE;

    robot.heading += angleDiff;

    // Update simulated position
    robot.x += robot.speed * std::cos(robot.heading);
    robot.y += robot.speed * std::sin(robot.heading);
}

//  PURE PURSUIT CONTROL LOOP (REAL ROBOT VERSION)
void runPurePursuit(const std::vector<Point>& path) {
    Robot robot = {0, 0, 0, BASE_SPEED};

    std::cout << "=== PURE PURSUIT (TankDrive + Adaptive Lookahead) ===\n";

    for (int step = 0; step < 500; ++step) {
        double L = computeAdaptiveLookahead(robot.speed);
        Point target = findLookaheadPoint(robot, path, L);

        double distToEnd = distance(robot, path.back());
        if (distToEnd < 1.0) {
            drivetrainTank.stop();
            std::cout << "Reached final destination!\n";
            break;
        }

        // Compute steering
        double targetAngle = angleToPoint(robot, target);
        double turnError = targetAngle - robot.heading;
        while (turnError > M_PI)  turnError -= 2 * M_PI;
        while (turnError < -M_PI) turnError += 2 * M_PI;

        // Control signals
        double forward = 100;              // Base RPM
        double turn = turnError * 120;     // Steering strength

        // Clamp turn power
        if (turn > 200) turn = 200;
        if (turn < -200) turn = -200;

        // Move robot using tank drive
        // drivetrainTank.driveWhileTurning(forward, turn);

        // Adjust speed near goal
        if (distToEnd < 10)
            robot.speed = 1.0;
        else
            robot.speed = BASE_SPEED;

        // Print debug info
        std::cout << "Step " << step
                  << " | Pos=(" << robot.x << ", " << robot.y << ")"
                  << " | Target=(" << target.x << ", " << target.y << ")"
                  << " | Lookahead=" << L
                  << " | Heading=" << robot.heading
                  << " | Turn=" << turn << "\n";

        pros::delay(20);  // 20ms for PROS loop timing
    }

    drivetrainTank.stop();
}

}  // namespace pure_pursuit