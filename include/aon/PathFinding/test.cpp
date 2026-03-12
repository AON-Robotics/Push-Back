#include <iostream>
#include <fstream>
#include "pathFidingConfig.hpp"
#include "rdp.hpp"
#include "spline.hpp"

int main() {
    RDP rdp;

    std::vector<Point> rawPath = {
        {0, 0},
    {2, 0},
    {4, 0},
    {6, 0},

    {7, 1},
    {8, 2},
    {9, 1},

    {10, 0},

    {12, 0},
    {14, 0},
    {16, 0},

    {17, -1},
    {18, -2},
    {19, -1},

    {20, 0},

    {20, 3},
    {20, 6},

    {18, 7},
    {16, 8},
    {14, 7},

    {12, 6},

    {10, 6},
    {8, 6},

    {6, 4},
    {4, 2},
    {2, 1},

    {0, 0}
    };

    // -----------------------------------------
    // 1. Create simple control points
    // Imagine these came from RDP
    // -----------------------------------------

    std::vector<Point> rdp_points = rdp.simplify(rawPath);

    // -----------------------------------------
    // 2. Create spline
    // -----------------------------------------

    Spline spline(rdp_points);

    // -----------------------------------------
    // 3. Generate evenly spaced path
    // (1 inch spacing)
    // -----------------------------------------

    auto path = spline.generateEvenPath(1.0);

    // -----------------------------------------
    // Export ORIGINAL PATH
    // -----------------------------------------

    std::ofstream origFile("Plot/raw_path.csv");

    for(auto& p : rawPath)
    origFile << p.x << "," << p.y << "\n";
    
    origFile.close();

    // -----------------------------------------
    // Export RDP points
    // -----------------------------------------

    std::ofstream controlFile("Plot/rdp_path.csv");

    for(auto& p : rdp_points)
        controlFile << p.x << "," << p.y << "\n";

    controlFile.close();

    std::cout << "Data exported.\n";

    // -----------------------------------------
    // Export dense spline samples
    // -----------------------------------------

    std::ofstream splineFile("Plot/spline_path.csv");

    double t0 = spline.startParameter();
    double t1 = spline.endParameter();

    for(int i=0;i<1000;i++)
    {
        double t = t0 + (t1 - t0) * (double(i)/1000);

        Point p = spline.position(t);

        splineFile << p.x << "," << p.y << "\n";
    }

    splineFile.close();

    // -----------------------------------------
    // Export evenly spaced path
    // -----------------------------------------

    std::ofstream pathFile("Plot/even_path.csv");

    for(auto& p : path)
        pathFile << p.x << "," << p.y << "\n";

    pathFile.close();


    return 0;
}