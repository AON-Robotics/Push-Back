#include "RDP.hpp"

double aon::RDP::perpendicularDistance(const Point& A, const Point& B, const Point& P) {
    double dx = B.x - A.x;
    double dy = B.y - A.y;

    // Handle degenerate case: A and B are the same
    if (dx == 0.0 && dy == 0.0) {
        double px = P.x - A.x;
        double py = P.y - A.y;
        return std::sqrt(px * px + py * py);
    }

    double numerator = std::abs(dx * (P.y - A.y) - dy * (P.x - A.x));
    double denominator = std::sqrt(dx * dx + dy * dy);
    return numerator / denominator;
}

void aon::RDP::rdpRecursive(const std::vector<Point>& points,
                       int startIndex,
                       int endIndex,
                       std::vector<bool>& keep)
{
    // Base case, if the index find each other, return
    if (endIndex <= startIndex + 1)
        return;

    const Point& A = points[startIndex];
    const Point& B = points[endIndex];

    double maxDistance = 0.0;    // farthest point distance
    int indexOfMax = startIndex; // position of farthest point

    // Go through all the points between start and end, and find the farthest
    for (int i = startIndex + 1; i < endIndex; i++) {
        double d = perpendicularDistance(A, B, points[i]);
        if (d > maxDistance) {
            maxDistance = d;
            indexOfMax = i;
        }
    }

    // If the farthest point is important, keep it
    if (maxDistance > ELIPSON) {
        keep[indexOfMax] = true;

        // Recursive call to the right and left side of the farthest point
        rdpRecursive(points, startIndex, indexOfMax, keep);
        rdpRecursive(points, indexOfMax, endIndex, keep);
    }
}

std::vector<aon::RDP::Point> aon::RDP::simplify(const std::vector<Point>& points) {
    if (points.size() < 2)
        return points;

    // Keep track of the relevant points
    std::vector<bool> keep(points.size(), false);

    // Always keep first and last
    keep[0] = true;
    keep[points.size() - 1] = true;

    // Evaluate which point is the farthest and if it is important
    rdpRecursive(points, 0, points.size() - 1, keep);

    // Put together all the relevant points
    std::vector<Point> result;

    for (size_t i = 0; i < points.size(); i++) {
        if (keep[i])
            result.push_back(points[i]);
    }

    return result;
}