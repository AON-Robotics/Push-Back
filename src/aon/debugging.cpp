#include "pure_pursuit.hpp"
#include "aon/tank-drive/tank-drive.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include <iostream>

using namespace aon; // all types (Robot, Point, etc.) are in aon

namespace aon {

void runPurePursuitDebug() {
    TankDrive drive;

    std::vector<Point> path = {
        {0, 0},
        {24, 0},
        {24, 24},
        {0, 24},
        {0, 0}
    };

    Robot robot = {0, 0, 0, BASE_SPEED};

    pros::lcd::initialize();
    pros::lcd::set_text(1, "Running Pure Pursuit Debug...");

    for (int step = 0; step < 200; ++step) {
        double L = computeAdaptiveLookahead(robot.speed);
        Point target = findLookaheadPoint(robot, path, L);
        double distToEnd = distance(robot, path.back());

        if (distToEnd < 1.0) {
            pros::lcd::set_text(2, "Reached final destination!");
            break;
        }

        updateRobot(robot, target);

        if (distToEnd < 10)
            robot.speed = 1.0;
        else
            robot.speed = BASE_SPEED;

        std::cout 
            << "Step " << step
            << " | Pos=(" << robot.x << ", " << robot.y << ")"
            << " | Target=(" << target.x << ", " << target.y << ")"
            << " | DistToEnd=" << distToEnd
            << " | Lookahead=" << L
            << " | Heading=" << robot.heading
            << " | Speed=" << robot.speed
            << std::endl;

        // Simple motion update (for simulation)
        double leftPower = robot.speed * (1 - robot.heading);
        double rightPower = robot.speed * (1 + robot.heading);
        drive.driveWhileTurning(leftPower, rightPower);

        pros::delay(100);
    }

    drive.stop();
}

} // namespace aon