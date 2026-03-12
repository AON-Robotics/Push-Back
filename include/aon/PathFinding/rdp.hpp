/**
 * Ramer-Douglas-Peucker
 * 
 * Take raw output of A* and simplify it.
 * 
 * Summary:
 * Draw a line between 2 points, find the farther perpendicular point to that line.
 * If the distance is greater than ELIPSON, its a important point so we keep it.
 * Else elimate the point
 * 
 */

#include <cmath>
#include <vector>
#include "pathFidingConfig.hpp"

class RDP {
    private:
    // Ask where to put the ELIPSON
    double ELIPSON = constant.elipson;
    
    public:
    RDP() {};

    /// @brief Calculate the perpendicular distance from line AB to point P
    /// @param A Start point
    /// @param B End point
    /// @param P Perpendicular point
    /// @return Distance from the line AB to P
    /// @note Use cross product method since using vectors.
    double perpendicularDistance(const Point& A, const Point& B, const Point& P);

    /// @brief Recursive call of rdp
    /// @param points List of points to evaluate which point is important
    /// @param startIndex First point
    /// @param endIndex Last point
    /// @param keep Bool list to keep track of which points are important
    void rdpRecursive(const std::vector<Point>& points,
                  int startIndex,
                  int endIndex,
                  std::vector<bool>& keep);
    
    /// @brief Call the recursive function and unify the important points
    /// @param points List of points to evaluate
    /// @return Simplify path
    std::vector<Point> simplify(const std::vector<Point>& points);


};