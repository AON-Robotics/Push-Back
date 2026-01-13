#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "astar.hpp"


int main() {
  Astar astar;

  // Needed to draw obstacles in plot
  std::vector<Astar::Point> obstacles;

  ////////////////////////////////////////
  // CREATION OF VEX FIELD
  ///////////////////////////////////////

  // VEX FIELD IN THIS GRID IS ORIENTED RED IN LEFT AND BLUE IN RIGHT

  // --------------------------
  // WALLS (PERIMETER)
  // --------------------------
  int TOP_WALL_R    = 0;
  int BOTTOM_WALL_R = astar.CELLS - 1;   // 143
  int LEFT_WALL_C   = 0;
  int RIGHT_WALL_C  = astar.CELLS - 1;   // 143

  // Top wall: r = 0, c = 0..143
  for (int c = 0; c < astar.CELLS; c++) {
    Astar::Point p{TOP_WALL_R, c};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }

  // Bottom wall: r = 143, c = 0..143
  for (int c = 0; c < astar.CELLS; c++) {
    Astar::Point p{BOTTOM_WALL_R, c};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }

  // Left wall: c = 0, r = 0..143
  for (int r = 0; r < astar.CELLS; r++) {
    Astar::Point p{r, LEFT_WALL_C};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }

  // Right wall: c = 143, r = 0..143
  for (int r = 0; r < astar.CELLS; r++) {
    Astar::Point p{r, RIGHT_WALL_C};
    astar.set_blocked(p, true);
    obstacles.push_back(p);
  }


  // LONG GOAL is 48.8 inches in length and 13.33 inches enclosed center
  // This long goal need spacing between wall
  // I will put for now 19 inches of space 
  int bottom_r = (astar.CELLS - 1) - 19; // This calculates the row of the bottom long goal 124
  int top_r = 19;
  int MID = astar.CELLS / 2;             // 72
  int L = 49;                            // Columns wide of long goal are 49 ~ 48.8
  int half = L /2;                      // 24
  int c0 = MID - half;                  // Start Column of long goal
  int c1 = MID + half;                  // End Column of long goal

  // Here we add the thickness which is around 6 inches 
  int thick = 6;
  int halfT = thick / 2;                // 3
  
  // r = 16,17,18,19,20,21 → 6 rows ✅ that is why the minus 1
  int top_r0 = top_r - halfT;           // top_r is the center row of the long goal 16
  int top_r1 = top_r + halfT - 1;       // 21

  int bot_r0 = bottom_r - halfT;      // Same for bottom long goal
  int bot_r1 = bottom_r + halfT - 1;

  // --------------------------
  // TOP LONG GOAL (6 INCHES THICK)
  // --------------------------
  for (int r = top_r0; r <= top_r1; r++) {
    for (int c = c0; c <= c1; c++) {
      Astar::Point p{r,c};
      astar.set_blocked(p, true);
      obstacles.push_back(p);
    }
  }

  // --------------------
  // BOTTOM LONG GOAL (6 inches thick)
  // --------------------
  for (int r = bot_r0; r <= bot_r1; r++) {                 // sweep thickness (rows)
    for (int c = c0; c <= c1; c++) {                       // sweep length (cols)
      Astar::Point p{r, c};
      astar.set_blocked(p, true);                        // set blocked in the grid
      obstacles.push_back(p);                              // save for blocked.csv
    }
  }
  
  // --------------------------
  // PARK ZONES (PENALTY AREA)
  // --------------------------
  int PARK_DEPTH = 17;                                   // From wall to field
  int PARK_HEIGHT = 19;                                 // Inches tall in our grid
  int PARK_HALF_H = PARK_HEIGHT / 2;

  int park_r0 = MID - PARK_HALF_H;
  int park_r1 = MID + PARK_HALF_H;

  // Penalty value (Tune depending on behavior)
  int PARK_PEN = 500;

  // Store penalty points for python plotting
  std::vector<Astar::Point> penalty_vector;

  // LEFT PARK ZONE: uses left wall as one side (c = 0 is wall already blocked)
  for (int r = park_r0; r <= park_r1; r++) {
    for (int c = 1; c <= PARK_DEPTH; c++) {
      Astar::Point p{r,c};
      astar.set_penalty(p,PARK_PEN);
      penalty_vector.push_back(p);
    }
  }

  // RIGHT park zone
  for (int r = park_r0; r <= park_r1; r++) {
    for (int c = (astar.CELLS - 2) - PARK_DEPTH; c <= astar.CELLS - 2; c++) {  // avoid wall col=143
      Astar::Point p{r, c};
      astar.set_penalty(p, PARK_PEN);
      penalty_vector.push_back(p);
    }
  }

  // --------------------------
  // CENTER GOALS (BLOCKED "X" IN CENTER)
  // --------------------------
  // Doc: Each Center Goal (Upper and Lower) is 22.6" (574mm) in length.
  // We approximate footprint as two diagonals crossing at the center.
  int CENTER_L = (int)std::round(22.6);     // ~23 cells
  int CENTER_HALF = CENTER_L / 2;           // ~11 cells each side of center

  // Thickness of the plastic/goal footprint in the grid (tune)
  int CENTER_THICK = 4;                     // inches/cells
  int CT = CENTER_THICK / 2;                // 2

  // Diagonal 1: top-left -> bottom-right
  for (int t = -CENTER_HALF; t <= CENTER_HALF; t++) {
    int r = MID + t;
    int c = MID + t;

    // thickness: block a small square around each diagonal point
    for (int dr = -CT; dr <= CT; dr++) {
      for (int dc = -CT; dc <= CT; dc++) {
        int rr = r + dr;
        int cc = c + dc;

        // stay inside the field (avoid walls)
        if (rr <= 0 || rr >= astar.CELLS - 1) continue;
        if (cc <= 0 || cc >= astar.CELLS - 1) continue;

        Astar::Point p{rr, cc};
        astar.set_blocked(p, true);
        obstacles.push_back(p);
      }
    }
  }

  // Diagonal 2: top-right -> bottom-left
  for (int t = -CENTER_HALF; t <= CENTER_HALF; t++) {
    int r = MID + t;
    int c = MID - t;

    for (int dr = -CT; dr <= CT; dr++) {
      for (int dc = -CT; dc <= CT; dc++) {
        int rr = r + dr;
        int cc = c + dc;

        if (rr <= 0 || rr >= astar.CELLS - 1) continue;
        if (cc <= 0 || cc >= astar.CELLS - 1) continue;

        Astar::Point p{rr, cc};
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
  int LOADER_DEPTH = 5;            // how far it sticks into the field
  int LOADER_THICK = 4;            // thickness in rows
  int LH = LOADER_THICK / 2;       // half thickness

  // We place loaders centered on the same "center rows" as the long goals
  // Top loader centered at top_r, bottom loader centered at bottom_r
  int top_loader_r0 = top_r - LH;
  int top_loader_r1 = top_r + LH - 1;

  int bot_loader_r0 = bottom_r - LH;
  int bot_loader_r1 = bottom_r + LH - 1;

  // Helper lambda to block a rectangle and push into obstacles list
  auto block_rect = [&](int r0, int r1, int c0, int c1) {
    for (int r = r0; r <= r1; r++) {
      for (int c = c0; c <= c1; c++) {

        // stay off the perimeter walls (they are already blocked)
        if (r <= 0 || r >= astar.CELLS - 1) continue;
        if (c <= 0 || c >= astar.CELLS - 1) continue;

        Astar::Point p{r, c};
        astar.set_blocked(p, true);
        obstacles.push_back(p);
      }
    }
  };

  // LEFT WALL loaders:
  // wall is at c=0, so loaders occupy c = 1..LOADER_DEPTH
  int left_c0 = 1;
  int left_c1 = LOADER_DEPTH;

  // RIGHT WALL loaders:
  // wall is at c=143, so loaders occupy c = (CELLS-1-LOADER_DEPTH) .. (CELLS-2)
  int right_c0 = (astar.CELLS - 1) - LOADER_DEPTH;  // 143 - 5 = 138
  int right_c1 = astar.CELLS - 2;                   // 142 (avoid wall)

  // 4 loaders total: top+bottom on left, top+bottom on right
  block_rect(top_loader_r0, top_loader_r1, left_c0, left_c1);     // left top loader
  block_rect(bot_loader_r0, bot_loader_r1, left_c0, left_c1);     // left bottom loader
  block_rect(top_loader_r0, top_loader_r1, right_c0, right_c1);   // right top loader
  block_rect(bot_loader_r0, bot_loader_r1, right_c0, right_c1);   // right bottom loader


  ////////////////////////////////////////
  // SIMPLE TEST (PLANE COORDS: 0,0 IS CENTER)
  ////////////////////////////////////////

  astar.set_robot_type(1);            // 1 = Big, 2 = Small

  int ROBOT_W = astar.get_robot_w();
  int ROBOT_H = astar.get_robot_h();

  // Plane coords (x,y): x right +, left -, y up +, down -
  int sx = -50;                       // 50 inches left of center
  int sy = 0;                         // same height as center
  int gx = 50;                        // 50 inches right of center
  int gy = 0;

  // Convert plane (x,y) -> grid (r,c)
  Astar::Point start = Astar::from_xy(sx, sy);
  Astar::Point goal  = Astar::from_xy(gx, gy);

  // Print conversion so you can verify the mapping
  std::cout << "Start XY (" << sx << "," << sy << ") -> grid (" << start.r << "," << start.c << ")\n";
  std::cout << "Goal  XY (" << gx << "," << gy << ") -> grid (" << goal.r  << "," << goal.c  << ")\n";

  // ---- Run ----
  auto path = astar.go(start, goal);

  if (path.empty()) {
    std::cout << "No path found.\n";
  } else {
    std::cout << "Path length: " << path.size() << "\n";
    std::cout << "Start grid: (" << path.front().r << "," << path.front().c << ")\n";
    std::cout << "Goal  grid: (" << path.back().r  << "," << path.back().c  << ")\n";
  }

  // ----------------------------------------------------
  // OPTIONAL TEST 2 (UNCOMMENT): goal INSIDE park zone
  // This should FORCE A* to enter the penalty area.
  // ----------------------------------------------------
  // Astar::Point goal_in_left_park{MID, 5};   // inside left park zone (c=1..PARK_DEPTH)
  // auto path2 = astar.go(start, goal_in_left_park);
  // std::cout << "Parking test path length: " << path2.size() << "\n";

  ////////////////////////////////////////
  // EXPORT CSV FOR YOUR PYTHON PLOT
  ///////////////////////////////////////

  // ---- Write CSV: path ----
  {
    std::ofstream out("path.csv");
    out << "r,c\n";
    for (const auto& p : path) out << p.r << "," << p.c << "\n";
  }

  // ---- Write CSV: blocked cells ----
  {
    std::ofstream out("blocked.csv");
    out << "r,c\n";
    for (const auto& p : obstacles) out << p.r << "," << p.c << "\n";
  }

    // ---- Write CSV: penalty cells ----
  {
    std::ofstream out("penalty.csv");
    out << "r,c\n";
    for (const auto& p : penalty_vector) out << p.r << "," << p.c << "\n";
  }

  // ---- Write CSV: robot footprint boxes (for visualization) ----
  // Each row is: center_r,center_c,w,h
  {
    std::ofstream out("robot_boxes.csv");
    out << "r,c,w,h\n";

    // Always draw at start + goal
    out << start.r << "," << start.c << "," << ROBOT_W << "," << ROBOT_H << "\n";
    out << goal.r  << "," << goal.c  << "," << ROBOT_W << "," << ROBOT_H << "\n";

    // Optional: draw every N steps along the path so you see it "moving"
    int step = 20; // change to 5/10 if you want more boxes
    for (int i = 0; i < (int)path.size(); i += step) {
      out << path[i].r << "," << path[i].c << "," << ROBOT_W << "," << ROBOT_H << "\n";
    }
  }

  std::cout << "Wrote path.csv and blocked.csv\n";
  return 0;  
}