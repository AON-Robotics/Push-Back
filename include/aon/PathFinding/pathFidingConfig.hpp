#pragma once
#include <cstddef>

struct Point {
    double x = 0; // row
    double y = 0; // column
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

struct Waypoint {
    Point p;              // grid point
    double heading_deg;   // where robot should face at this point
};

struct Config {
    double elipson = 0.7;       // How far do the point have to be to considered important
    double alpha = 0.5;         // Exponent used for parameter spacing
    int arcSamples = 1000;      // Number of samples used to approximate length
    double spacing = 6.0;       // Space between points in the path
};

struct ClosestPointResult {
    Point point;        // closest point on the path
    double distance;    // distance from robot to path
    size_t segment;     // segment index (path[segment] -> path[segment+1])
};

static Config constant;