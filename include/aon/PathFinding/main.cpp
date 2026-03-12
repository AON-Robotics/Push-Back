#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "astar.hpp"
#include "pathFidingConfig.hpp"
#include "rdp.hpp"
#include "spline.hpp"


int main() {
  Astar astar;
  RDP rdp;

  // Needed to draw obstacles in plot
  std::vector<Point> obstacles;

  ////////////////////////////////////////
  // CREATION OF VEX FIELD
  ///////////////////////////////////////

  // VEX FIELD IN THIS GRID IS ORIENTED RED IN LEFT AND BLUE IN RIGHT

  // --------------------------
  // WALLS (PERIMETER)
  // --------------------------
  double TOP_WALL_R    = 0;
  double BOTTOM_WALL_R = astar.CELLS - 1;   // 143
  double LEFT_WALL_C   = 0;
  double RIGHT_WALL_C  = astar.CELLS - 1;   // 143

  // Top wall: y = 0, x = 0..143
  for (double x = 0; x < astar.CELLS; x++) {
    Point p{TOP_WALL_R, x};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }

  // Bottom wall: y = 143, x = 0..143
  for (double x = 0; x < astar.CELLS; x++) {
    Point p{BOTTOM_WALL_R, x};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }

  // Left wall: x = 0, y = 0..143
  for (double y = 0; y < astar.CELLS; y++) {
    Point p{y, LEFT_WALL_C};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }

  // Right wall: x = 143, y = 0..143
  for (double y = 0; y < astar.CELLS; y++) {
    Point p{y, RIGHT_WALL_C};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }


  // LONG GOAL is 48.8 inches in length and 13.33 inches enclosed center
  // This long goal need spacing between wall
  // I will put for now 19 inches of space 
  double bottom_r = (astar.CELLS - 1) - 19; // This calculates the row of the bottom long goal 124
  double top_r = 19;
  double MID = astar.CELLS / 2;             // 72
  double L = 49;                            // Columns wide of long goal are 49 ~ 48.8
  double half = L /2;                      // 24
  double c0 = MID - half;                  // Start Column of long goal
  double c1 = MID + half;                  // End Column of long goal

  // Here we add the thickness which is around 6 inches 
  double thick = 6;
  double halfT = thick / 2;                // 3
  
  // y = 16,17,18,19,20,21 → 6 rows ✅ that is why the minus 1
  double top_r0 = top_r - halfT;           // top_r is the center row of the long goal 16
  double top_r1 = top_r + halfT - 1;       // 21

  double bot_r0 = bottom_r - halfT;      // Same for bottom long goal
  double bot_r1 = bottom_r + halfT - 1;

  // --------------------------
  // TOP LONG GOAL (6 INCHES THICK)
  // --------------------------
  for (double y = top_r0; y <= top_r1; y++) {
    for (double x = c0; x <= c1; x++) {
      Point p{y,x};
      astar.set_blocked(p, true);
      obstacles.push_back(p);
    }
  }

  // --------------------
  // BOTTOM LONG GOAL (6 inches thick)
  // --------------------
  for (double y = bot_r0; y <= bot_r1; y++) {                 // sweep thickness (rows)
    for (double x = c0; x <= c1; x++) {                       // sweep length (cols)
      Point p{y, x};
      astar.set_blocked(p, true);                        // set blocked in the grid
      obstacles.push_back(p);                              // save for blocked.csv
    }
  }
  
  // --------------------------
  // PARK ZONES (PENALTY AREA)
  // --------------------------
  double PARK_DEPTH = 17;                                   // From wall to field
  double PARK_HEIGHT = 19;                                 // Inches tall in our grid
  double PARK_HALF_H = PARK_HEIGHT / 2;

  double park_r0 = MID - PARK_HALF_H;
  double park_r1 = MID + PARK_HALF_H;

  // Penalty value (Tune depending on behavior)
  double PARK_PEN = 500;

  // Store penalty points for python plotting
  std::vector<Point> penalty_vector;

  // LEFT PARK ZONE: uses left wall as one side (x = 0 is wall already blocked)
  for (double y = park_r0; y <= park_r1; y++) {
    for (double x = 1; x <= PARK_DEPTH; x++) {
      Point p{y,x};
      astar.set_penalty(p,PARK_PEN);
      penalty_vector.push_back(p);
    }
  }

  // RIGHT park zone
  for (double y = park_r0; y <= park_r1; y++) {
    for (double x = (astar.CELLS - 2) - PARK_DEPTH; x <= astar.CELLS - 2; x++) {  // avoid wall col=143
      Point p{y, x};
      astar.set_penalty(p, PARK_PEN);
      penalty_vector.push_back(p);
    }
  }

  // --------------------------
  // CENTER GOALS (BLOCKED "X" IN CENTER)
  // --------------------------
  // Doc: Each Center Goal (Upper and Lower) is 22.6" (574mm) in length.
  // We approximate footprint as two diagonals crossing at the center.
  double CENTER_L = (double)std::round(22.6);     // ~23 cells
  double CENTER_HALF = CENTER_L / 2;           // ~11 cells each side of center

  // Thickness of the plastic/goal footprint in the grid (tune)
  double CENTER_THICK = 4;                     // inches/cells
  double CT = CENTER_THICK / 2;                // 2

  // Diagonal 1: top-left -> bottom-right
  for (double t = -CENTER_HALF; t <= CENTER_HALF; t++) {
    double y = MID + t;
    double x = MID + t;

    // thickness: block a small square around each diagonal point
    for (double dr = -CT; dr <= CT; dr++) {
      for (double dc = -CT; dc <= CT; dc++) {
        double rr = y + dr;
        double cc = x + dc;

        // stay inside the field (avoid walls)
        if (rr <= 0 || rr >= astar.CELLS - 1) continue;
        if (cc <= 0 || cc >= astar.CELLS - 1) continue;

        Point p{rr, cc};
        astar.set_blocked(p, true);
        obstacles.push_back(p);
      }
    }
  }

  // Diagonal 2: top-right -> bottom-left
  for (double t = -CENTER_HALF; t <= CENTER_HALF; t++) {
    double y = MID + t;
    double x = MID - t;

    for (double dr = -CT; dr <= CT; dr++) {
      for (double dc = -CT; dc <= CT; dc++) {
        double rr = y + dr;
        double cc = x + dc;

        if (rr <= 0 || rr >= astar.CELLS - 1) continue;
        if (cc <= 0 || cc >= astar.CELLS - 1) continue;

        Point p{rr, cc};
        astar.set_blocked(p, true);
        obstacles.push_back(p);
      }
    }
  }


  // --------------------------
  // LOADERS (BLOCKED RECTANGLES, STUCK TO WALL)
  // --------------------------
  // Approx from manual/map:
  // depth into field ~ 5"
  // thickness (along rows) ~ 4"
  double LOADER_DEPTH = 5;            // how far it sticks into the field
  double LOADER_THICK = 4;            // thickness in rows
  double LH = LOADER_THICK / 2;       // half thickness

  // We place loaders centered on the same "center rows" as the long goals
  // Top loader centered at top_r, bottom loader centered at bottom_r
  double top_loader_r0 = top_r - LH;
  double top_loader_r1 = top_r + LH - 1;

  double bot_loader_r0 = bottom_r - LH;
  double bot_loader_r1 = bottom_r + LH - 1;

  // Helper lambda to block a rectangle and push into obstacles list
  auto block_rect = [&](double r0, double r1, double c0, double c1) {
    for (double y = r0; y <= r1; y++) {
      for (double x = c0; x <= c1; x++) {

        // stay off the perimeter walls (they are already blocked)
        if (y <= 0 || y >= astar.CELLS - 1) continue;
        if (x <= 0 || x >= astar.CELLS - 1) continue;

        Point p{y, x};
        astar.set_blocked(p, true);
        obstacles.push_back(p);
      }
    }
  };

  // LEFT WALL loaders:
  // wall is at x=0, so loaders occupy x = 1..LOADER_DEPTH
  double left_c0 = 1;
  double left_c1 = LOADER_DEPTH;

  // RIGHT WALL loaders:
  // wall is at x=143, so loaders occupy x = (CELLS-1-LOADER_DEPTH) .. (CELLS-2)
  double right_c0 = (astar.CELLS - 1) - LOADER_DEPTH;  // 143 - 5 = 138
  double right_c1 = astar.CELLS - 2;                   // 142 (avoid wall)

  // 4 loaders total: top+bottom on left, top+bottom on right
  block_rect(top_loader_r0, top_loader_r1, left_c0, left_c1);     // left top loader
  block_rect(bot_loader_r0, bot_loader_r1, left_c0, left_c1);     // left bottom loader
  block_rect(top_loader_r0, top_loader_r1, right_c0, right_c1);   // right top loader
  block_rect(bot_loader_r0, bot_loader_r1, right_c0, right_c1);   // right bottom loader


  ////////////////////////////////////////
  // SIMPLE TEST (PLANE COORDS: 0,0 IS CENTER)
  ////////////////////////////////////////

  astar.set_robot_type(1);            // 1 = Big, 2 = Small

  double ROBOT_W = astar.get_robot_w();
  double ROBOT_H = astar.get_robot_h();

  // Plane coords (x,y): x right +, left -, y up +, down -
  double sx = -50;                       // 50 inches left of center
  double sy = 0;                         // same height as center
  double gx = 50;                        // 50 inches right of center
  double gy = 0;

  // Convert plane (x,y) -> grid (y,x)
  Point start = Astar::from_xy(sx, sy);
  Point goal  = Astar::from_xy(gx, gy);

  // Print conversion so you can verify the mapping
  std::cout << "Start XY (" << sx << "," << sy << ") -> grid (" << start.y << "," << start.x << ")\n";
  std::cout << "Goal  XY (" << gx << "," << gy << ") -> grid (" << goal.y  << "," << goal.x  << ")\n";

  // ---- Run ----
  // -----------------------------------------
  // 1. Create path with A*
  // -----------------------------------------
  auto rawPath = astar.go(start, goal);

  // -----------------------------------------
  // 2. Create simple control points
  // -----------------------------------------

  std::vector<Point> rdp_points = rdp.simplify(rawPath);

  // -----------------------------------------
  // 3. Create spline
  // -----------------------------------------
  Spline spline(rdp_points);
  
  // -----------------------------------------
  // 5. Generate evenly spaced path
  // (1 inch spacing)
  // -----------------------------------------
  auto path = spline.generateEvenPath(1.0);

  // -----------------------------------------
  // 4. Add heading
  // -----------------------------------------
  auto waypoints = astar.add_headings(path);

  // Write those heading in the ouput csv
  for (double i = 0; i < (double)waypoints.size(); i++) {
  std::cout << i << ": (" << waypoints[i].p.y << "," << waypoints[i].p.x
            << ") heading=" << waypoints[i].heading_deg << " deg\n";
  }

  if (path.empty()) {
    std::cout << "No path found.\n";
  } else {
    std::cout << "Path length: " << path.size() << "\n";
    std::cout << "Start grid: (" << path.front().y << "," << path.front().x << ")\n";
    std::cout << "Goal  grid: (" << path.back().y  << "," << path.back().x  << ")\n";
  }

  // ----------------------------------------------------
  // OPTIONAL TEST 2 (UNCOMMENT): goal INSIDE park zone
  // This should FORCE A* to enter the penalty area.
  // ----------------------------------------------------
  // Point goal_in_left_park{MID, 5};   // inside left park zone (x=1..PARK_DEPTH)
  // auto path2 = astar.go(start, goal_in_left_park);
  // std::cout << "Parking test path length: " << path2.size() << "\n";

  ////////////////////////////////////////
  // EXPORT CSV FOR YOUR PYTHON PLOT
  ///////////////////////////////////////

  // ---- Write CSV: path ----
  {
    std::ofstream out("Plot/path.csv");
    out << "y,x\n";
    for (const auto& p : path) out << p.y << "," << p.x << "\n";
  }

  // ---- Write CSV: blocked cells ----
  {
    std::ofstream out("Plot/blocked.csv");
    out << "y,x\n";
    for (const auto& p : obstacles) out << p.y << "," << p.x << "\n";
  }

    // ---- Write CSV: penalty cells ----
  {
    std::ofstream out("Plot/penalty.csv");
    out << "y,x\n";
    for (const auto& p : penalty_vector) out << p.y << "," << p.x << "\n";
  }

  // ---- Write CSV: robot footprint boxes (for visualization) ----
  // Each row is: center_r,center_c,w,h
  {
    std::ofstream out("Plot/robot_boxes.csv");
    out << "y,x,w,h\n";

    // Always draw at start + goal
    out << start.y << "," << start.x << "," << ROBOT_W << "," << ROBOT_H << "\n";
    out << goal.y  << "," << goal.x  << "," << ROBOT_W << "," << ROBOT_H << "\n";

    // Optional: draw every N steps along the path so you see it "moving"
    double step = 20; // change to 5/10 if you want more boxes
    for (double i = 0; i < (double)path.size(); i += step) {
      out << path[i].y << "," << path[i].x << "," << ROBOT_W << "," << ROBOT_H << "\n";
    }
  }

  std::cout << "Wrote path.csv and blocked.csv\n";

  // -----------------------------------------
    // Export RDP points
    // -----------------------------------------

    std::ofstream controlFile("Plot/rdp_path.csv");

    for(auto& p : rdp_points)
        controlFile << p.y << "," << p.x << "\n";

    controlFile.close();

    // -----------------------------------------
    // Export dense spline samples
    // -----------------------------------------

    std::ofstream splineFile("Plot/spline_path.csv");

    double t0 = spline.startParameter();
    double t1 = spline.endParameter();

    for(double i=0;i<1000;i++)
    {
        double t = t0 + (t1 - t0) * (double(i)/1000);

        Point p = spline.position(t);

        splineFile << p.y << "," << p.x << "\n";
    }

    splineFile.close();

    // -----------------------------------------
    // Export evenly spaced path
    // -----------------------------------------

    std::ofstream pathFile("Plot/even_path.csv");

    for(auto& p : path)
        pathFile << p.y << "," << p.x << "\n";

    pathFile.close();

  return 0;  
}