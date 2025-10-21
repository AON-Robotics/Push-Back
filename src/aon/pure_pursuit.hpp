#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

namespace aon {
//  STRUCTS & CONSTANTS
struct Point {
    double x, y;
};

struct Robot {
    double x, y;     // current position
    double heading;  // current heading angle (radians)
    double speed;    // current speed
};

// Base configuration constants
const double MIN_LOOKAHEAD = 3.5;   // inches (minimum distance)
const double MAX_LOOKAHEAD = 6.5;  // inches (maximum distance)
const double SPEED_SCALE   = 0.7;   // scaling factor for adaptive lookahead
const double MAX_TURN_RATE = 0.15;  // radians per frame (smooth turning)
const double BASE_SPEED    = 2.0;   // base forward speed 

//  FUNCTION DECLARATIONS
double distance(const Point& a, const Point& b);
double distance(const Robot& r, const Point& p);
double angleToPoint(const Robot& robot, const Point& target);
double computeAdaptiveLookahead(double speed);
Point findLookaheadPoint(const Robot& robot, const std::vector<Point>& path, double lookahead);
void updateRobot(Robot& robot, const Point& target);
void runPurePursuit(const std::vector<Point>& path);
}