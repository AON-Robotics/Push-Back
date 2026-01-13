// astar.cpp
#include "astar.hpp"
#include <queue>
#include <limits>
#include <algorithm>

std::vector<Astar::Point>
Astar::astar(const CostGrid& grid, const Point& start, const Point& goal) const {
  // If start or goal is out of bounds we return empty vector
  if (!grid.in_bounds(start.r, start.c) or !grid.in_bounds(goal.r,goal.c)) {
    return {};
  }
  // If start or goal cannot fit (robot footprint collides) return empty
  if (!fits_rect(grid, start) || !fits_rect(grid, goal)) {
    return {};
  }
  // If start == goal return start
  if (start == goal) {
    return {start};
  }

  // Take start and goal indexes
  int start_idx = grid.idx(start.r, start.c);
  int goal_idx = grid.idx(goal.r,goal.c);

  // Resources needed for runtime computation
  const int N = grid.rows * grid.cols;
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> gscore(N, INF); // We initialize all values to INF at begining
  std::vector<int> parent(N, -1);
  std::vector<std::uint8_t> closed(N, 0);

  // Open List item
  struct OpenItem {
    int idx;
    double f;
    double g;
  };
  struct Cmp {
    bool operator()(const OpenItem& a, const OpenItem& b) const {
      return a.f > b.f; // We treat smaller f as higer priority
    }
  };

  // Priority Queue itself
  std::priority_queue<OpenItem, std::vector<OpenItem>,Cmp> open;

  // Initialization first node g_score of 0.0
  gscore[start_idx] = 0.0;
  open.push({start_idx, h_cost(start,goal), 0.0});

  // Arrays for 8 directional field exploration
  static constexpr int DR[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
  static constexpr int DC[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

  // Magic happens here
  while (!open.empty()) {
    // Get lowest f in Open list
    OpenItem item = open.top();
    open.pop();

    // Item already expanded ignore, since we use lazy duplicate technique this could happen
    if (closed[item.idx]) {
      continue;
    }
    // If we later found a better route to this same cell/node we ignore the current one.
    if (item.g != gscore[item.idx]) {
      continue;
    }

    // For this node we mark is as closed
    closed[item.idx] = 1;

    // Is this node the goal?
    if (item.idx == goal_idx) {
      break;
    }

    // Get 2D from the idx we just pop
    int curr_r = item.idx / grid.cols;
    int curr_c = item.idx % grid.cols;
    Point curr_point{curr_r,curr_c};

    // Iterate over neighbors
    for(int k = 0; k < 8; k++) {
      // Get neighbor
      Point nb{curr_point.r + DR[k], curr_point.c + DC[k]};

      // Basic checkings
      if (!grid.in_bounds(nb.r, nb.c)) continue;
      // Robot footprint collision check
      if (!fits_rect(grid, nb)) continue;

      int nb_idx = grid.idx(nb.r, nb.c);
      if (closed[nb_idx]) continue;

      // Computing tentative g
      double tentative = g_cost(grid, curr_point, nb, gscore[item.idx]);

      // If this path is better we update
      if (tentative < gscore[nb_idx]) {
        gscore[nb_idx] = tentative;
        parent[nb_idx] = item.idx;
        double f = tentative + h_cost(nb, goal);
        open.push({nb_idx, f, tentative});
      }
    }
  }

  // Was the goal reached? if not return empty path
  if (!closed[goal_idx]) return {};

  // Construct the path
  std::vector<Point> path;
  int at = goal_idx;
  while (at != -1) {
    int r = at / grid.cols;
    int c = at % grid.cols;
    path.push_back(Point{r, c});
    at = parent[at];
  }
  std::reverse(path.begin(), path.end());
  return path;
}
