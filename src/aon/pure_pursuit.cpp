#include "pure_pursuit.hpp"
#include "globals.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

using namespace aon;

namespace aon {

// ===========================================================
//   Enable / Disable Simulation Mode
// ===========================================================
#define PURE_PURSUIT_SIMULATION 1
// 0 = Use odometry + tank drive (real robot)
// 1 = Use internal simulated robot model (no motors needed)

// ===========================================================
// Helpers
// ===========================================================

template <typename T>
T clampValue(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

double distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double distance(const Robot& r, const Point& p) {
    return std::sqrt(std::pow(p.x - r.x, 2) + std::pow(p.y - r.y, 2));
}

double angleToPoint(const Robot& robot, const Point& target) {
    return std::atan2(target.y - robot.y, target.x - robot.x);
}

// ===========================================================
// Adaptive Lookahead
// ===========================================================

double computeAdaptiveLookahead(double speed) {
    double L = MIN_LOOKAHEAD + SPEED_SCALE * speed;
    return clampValue(L, MIN_LOOKAHEAD, MAX_LOOKAHEAD);
}

// ===========================================================
// Lookahead Point
// ===========================================================

Point findLookaheadPoint(const Robot& robot, const std::vector<Point>& path, double lookahead) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double d1 = distance(robot, path[i]);
        double d2 = distance(robot, path[i + 1]);

        if (d1 < lookahead && d2 >= lookahead) {
            double t = (lookahead - d1) / (d2 - d1);
            return {
                path[i].x + t * (path[i + 1].x - path[i].x),
                path[i].y + t * (path[i + 1].y - path[i].y)
            };
        }
    }
    return path.back();
}

// ===========================================================
// Simulation Mode: Update Robot Pose Internally
// ===========================================================

void updateRobot(Robot& robot, const Point& target) {
    double targetAngle = angleToPoint(robot, target);

    double angleDiff = targetAngle - robot.heading;
    while (angleDiff >  M_PI) angleDiff -= 2*M_PI;
    while (angleDiff < -M_PI) angleDiff += 2*M_PI;

    if (angleDiff >  MAX_TURN_RATE) angleDiff = MAX_TURN_RATE;
    if (angleDiff < -MAX_TURN_RATE) angleDiff = -MAX_TURN_RATE;

    robot.heading += angleDiff;
    robot.x += robot.speed * std::cos(robot.heading);
    robot.y += robot.speed * std::sin(robot.heading);
}

// ===========================================================
// Pure Pursuit Main Function
// ===========================================================

void runPurePursuit(const std::vector<Point>& path) {

    Robot robot;

#if PURE_PURSUIT_SIMULATION == 1
    // =======================================================
    //   SIMULATION MODE
    // =======================================================
    robot.x = 0;
    robot.y = 0;
    robot.heading = 0;
    robot.speed = BASE_SPEED;

    std::cout << "[SIMULATION MODE ON]\n";

#else
    // =======================================================
    //   REAL ROBOT MODE (ODOMETRY + TANK DRIVE)
    // =======================================================
    Vector pos = odometry::GetPosition();
    robot.x = pos.GetX();
    robot.y = pos.GetY();
    robot.heading = odometry::GetRadians();
    robot.speed = BASE_SPEED;

    std::cout << "[REAL ROBOT MODE]\n";
#endif

    const double END_THRESHOLD = 1.0;  // inches
    const double LOOP_DT = 20;         // ms

    while (true) {

#if PURE_PURSUIT_SIMULATION == 0
        // Update from real odometry
        Vector pos = odometry::GetPosition();
        robot.x = pos.GetX();
        robot.y = pos.GetY();
        robot.heading = odometry::GetRadians();
#endif

        // Distance to the final point
        double distToEnd = distance(robot, path.back());
        if (distToEnd < END_THRESHOLD) {
#if PURE_PURSUIT_SIMULATION == 0
            drivetrainTank.stop();
#endif
            std::cout << "Reached destination!\n";
            break;
        }

        // Compute adaptive L
        double L = computeAdaptiveLookahead(robot.speed);

        // Lookahead target
        Point target = findLookaheadPoint(robot, path, L);

        // Steering logic
        double targetAngle = angleToPoint(robot, target);
        double turnError = targetAngle - robot.heading;

        while (turnError >  M_PI) turnError -= 2*M_PI;
        while (turnError < -M_PI) turnError += 2*M_PI;

        double forward = 100;               // constant forward
        double turn = turnError * 120;      // proportional turn
        turn = std::clamp(turn, -200.0, 200.0);

#if PURE_PURSUIT_SIMULATION == 0
        // Move actual robot
        drivetrainTank.driveWhileTurning(forward, turn);
#else
        // Update simulated model
        updateRobot(robot, target);
#endif

        // Slow down near the goal
        robot.speed = (distToEnd < 10) ? 1.0 : BASE_SPEED;

        pros::delay(LOOP_DT);
    }

#if PURE_PURSUIT_SIMULATION == 0
    drivetrainTank.stop();
#endif
}

} // namespace aon