#include "spline.hpp"

Point Spline::hermite(const Point& P0, const Point& P1, const Point& m0, const Point& m1,
              double t0, double t1, double t_query) {
        
    // Normalize parameter into [0,1]
    double tau = (t_query - t0)/(t1 - t0);

    // Hermite basis functions
    double h00 = 2*tau*tau*tau - 3*tau*tau + 1;
    double h10 = tau*tau*tau - 2*tau*tau + tau;
    double h01 = -2*tau*tau*tau + 3*tau*tau;
    double h11 = tau*tau*tau - tau*tau;

    Point result;
    result.x = h00*P0.x + h10*(t1-t0)*m0.x + h01*P1.x + h11*(t1-t0)*m1.x;
    result.y = h00*P0.y + h10*(t1-t0)*m0.y + h01*P1.y + h11*(t1-t0)*m1.y;
    return result;
}

Point Spline::position(double t_query) {
    int n = points.size();
    if(t_query <= t[0]) return points[0];
    if(t_query >= t[n-1]) return points[n-1];
    auto it = std::upper_bound(t.begin(), t.end(), t_query);
    int i = std::distance(t.begin(), it) - 1;
    return hermite(points[i], points[i+1], tangents[i], tangents[i+1], t[i], t[i+1], t_query);
}

Point Spline::velocity(double t_query, double dt) {
    Point p1 = position(t_query + dt);
    Point p0 = position(t_query - dt);
    return Point{ (p1.x-p0.x)/(2*dt), (p1.y-p0.y)/(2*dt) };
}

void Spline::buildArcLengthTable(){

    arcLength.clear();
    arcLength.resize(constant.arcSamples + 1);

    double t_start = t.front();
    double t_end   = t.back();

    double total = 0.0;

    // First point has zero arc length
    arcLength[0] = 0.0;

    Point prev = position(t_start);

    for(int i = 1; i <= constant.arcSamples; i++)
    {
        // Uniformly sample parameter space
        double ti = t_start + (t_end - t_start) * (double(i) / constant.arcSamples);

        Point curr = position(ti);

        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;

        double segmentLength = std::sqrt(dx*dx + dy*dy);
        total += segmentLength;
        arcLength[i] = total;

        prev = curr;
    }
}

double Spline::totalLength() const
{
    if(arcLength.empty()) return 0.0;
    return arcLength.back();
}

double Spline::parameterAtArcLength(double s) const
{
    if(arcLength.empty()) return t.front();

    if(s <= 0.0) return t.front();
    if(s >= arcLength.back()) return t.back();

    // Binary search for interval
    auto it = std::lower_bound(arcLength.begin(), arcLength.end(), s);
    int index = std::distance(arcLength.begin(), it);

    if(index == 0) return t.front();

    // Linear interpolation between neighboring samples
    double s0 = arcLength[index - 1];
    double s1 = arcLength[index];

    double denom = s1 - s0;
    double ratio = (denom > 1e-9) ? (s - s0) / denom : 0.0;

    double t_start = t.front();
    double t_end   = t.back();

    double tau0 = double(index - 1) / constant.arcSamples;
    double tau1 = double(index)     / constant.arcSamples;

    double param0 = t_start + (t_end - t_start) * tau0;
    double param1 = t_start + (t_end - t_start) * tau1;

    return param0 + ratio * (param1 - param0);
}

std::vector<Point> Spline::generateEvenPath(double spacing)
{
    std::vector<Point> result;
    
    buildArcLengthTable(); // Prepare the points of the spline
        
    double total = totalLength();
        
    int numPoints = static_cast<int>(total / spacing);
    result.reserve(numPoints + 2);

    for(int i = 0; i <= numPoints; i++)
    {
        double s = i * spacing;

        double param = parameterAtArcLength(s);

        result.push_back(position(param));
    }

    result.push_back(position(t.back())); 
    return result;
}

ClosestPointResult Spline::findClosestPointOnPath(const Point& robot, const std::vector<Point>& path){
    ClosestPointResult result;
    result.distance = std::numeric_limits<double>::max();
    result.segment = 0;

    for (size_t i = 0; i < path.size() - 1; i++) {

        const Point& A = path[i];
        const Point& B = path[i + 1];

        double ABx = B.x - A.x;
        double ABy = B.y - A.y;

        double APx = robot.x - A.x;
        double APy = robot.y - A.y;

        double ab2 = ABx * ABx + ABy * ABy;

        if (ab2 == 0) continue; // safety check

        double t = (APx * ABx + APy * ABy) / ab2;

        // clamp projection to segment
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        Point candidate;
        candidate.x = A.x + ABx * t;
        candidate.y = A.y + ABy * t;

        double dx = robot.x - candidate.x;
        double dy = robot.y - candidate.y;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist < result.distance) {
            result.distance = dist;
            result.point = candidate;
            result.segment = i;
        }
    }

    return result;
}


