// astar.hpp
#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstdlib>

// --------------------
// A STAR CLASS
// --------------------
class Astar {
public:
  // 1 foot = 12 inches
  static constexpr int INCH = 12;                 // inches per foot
  static constexpr int RESOLUTION = 1;            // inches per cell
  static constexpr int FIELD_SIZE_INCHES = INCH * 12;  // 12 ft * 12 in/ft = 144 in
  static constexpr int CELLS = FIELD_SIZE_INCHES / RESOLUTION; // cells per side

  // Robot selection:
  // 1 = big robot
  // 2 = small robot
  void set_robot_type(int type) {
    if (type == 1) {                // BIG
      robot_w = b_robot_w;
      robot_h = b_robot_h;
    } else {                        // default SMALL (type == 2 or anything else)
      robot_w = s_robot_w;
      robot_h = s_robot_h;
    }
  }

  int get_robot_w() const { return robot_w; };
  int get_robot_h() const { return robot_h; };

  struct Point {
    int r = 0; // row
    int c = 0; // column
    bool operator==(const Point& other) const { return r == other.r && c == other.c; }
  };

  Astar() { set_robot_type(1); } // Default Big Robot selected

  // (Optional helpers) so you can add obstacles/penalties from outside
  void set_blocked(Point p, bool v = true) { field.set_blocked(p.r, p.c, v); }
  void set_penalty(Point p, int pen) { field.set_penalty(p.r, p.c, pen); }

  // Call A* algorithm using start + goal provided by caller
  std::vector<Point> go(Point start, Point goal) const { return astar(field, start, goal); }

private:

  // Active robot footprint used by collision checks
  int robot_w = 1;     // width in cells/inches
  int robot_h = 1;     // height in cells/inches

  // Big Robot Measurments
  int b_robot_w = 19;  // rounded from 18 and something inches
  int b_robot_h = 23;  // rounded from 22 inches and somehting

  // Small Robot Measurments
  int s_robot_w = 15;  // rounded from 14 and somehting
  int s_robot_h = 16;  // rouded from 15 and somehting

  struct CostGrid {
    int rows = 0;
    int cols = 0;
    std::vector<int> penalty_vector;          // per-cell penalty
    std::vector<std::uint8_t> blocked_vector; // 0 free, 1 blocked

    CostGrid(int r, int c)
      : rows(r), cols(c),
        penalty_vector(r * c, 0),
        blocked_vector(r * c, 0) {}

    // Return 1D index from 2D given index
    int idx(int r, int c) const { return r * cols + c; }
    // Return True if given cell is in grid bounds
    bool in_bounds(int r, int c) const { return r >= 0 && c >= 0 && r < rows && c < cols; }
    // Return True is given cell is blocked
    bool is_blocked(int r, int c) const { return blocked_vector[idx(r, c)] == 1; }
    int  penalty(int r, int c) const { return penalty_vector[idx(r, c)]; }

    void set_blocked(int r, int c, bool v = true) { blocked_vector[idx(r, c)] = v ? 1 : 0; }
    void set_penalty(int r, int c, int p) { penalty_vector[idx(r, c)] = p; }

    // Move cost between adjacent cells (8-neighbors):
    // straight = RESOLUTION, diagonal = RESOLUTION*sqrt(2)
    double move_cost(const Point& a, const Point& b) const {
      int dr = std::abs(a.r - b.r);
      int dc = std::abs(a.c - b.c);
      if (dr + dc == 1) return double(RESOLUTION);      // straight
      return double(RESOLUTION) * std::sqrt(2.0);       // diagonal
    }
  };

  CostGrid field{CELLS, CELLS}; // e.g., 144 x 144 when RESOLUTION=1 inch/cell

  // True if robot rectangle centered at p can fit without touching blocked cells.
  // We allow 0° or 90° rotation: (w,h) OR (h,w) must fit.
  bool fits_rect(const CostGrid& grid, const Point& p) const {

    // Local lambda to test one (w,h) orientation
    auto fits_wh = [&](int w, int h) -> bool {
      int hw = w / 2;   // half width (columns)
      int hh = h / 2;   // half height (rows)

      for (int dr = -hh; dr <= hh; dr++) {
        for (int dc = -hw; dc <= hw; dc++) {
          int rr = p.r + dr;
          int cc = p.c + dc;

          // out of bounds = collision
          if (!grid.in_bounds(rr, cc)) return false;

          // any blocked cell inside footprint = collision
          if (grid.is_blocked(rr, cc)) return false;
        }
      }
      return true;
    };

    // Accept if either orientation fits
    return fits_wh(robot_w, robot_h) || fits_wh(robot_h, robot_w);
  }


  // Heuristic: Euclidean distance (in inches, consistent with move_cost)
  double h_cost(const Point& n, const Point& goal) const {
    double dr = double(n.r - goal.r);
    double dc = double(n.c - goal.c);
    return double(RESOLUTION) * std::sqrt(dr * dr + dc * dc);
  }

  // Tentative g for neighbor: g(curr) + move + penalty(neighbor)
  double g_cost(const CostGrid& field, const Point& curr, const Point& nb, double g_curr) const {
    return g_curr + field.move_cost(curr, nb) + double(field.penalty(nb.r, nb.c));
  }

  std::vector<Point> astar(const CostGrid& grid, const Point& start, const Point& goal) const;
};
