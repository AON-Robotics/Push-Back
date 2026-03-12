/**
 * Clamped Catmull–Rom Spline
 * 
 * Take points from RDP and create smooth path along the points. 
 * Then, with an arc length sampling, we create even spaces between the points.
 * 
 * Summary:
 *  
 */

#include <vector>
#include <cmath>
#include <array>
// #include <algorithm>
#include "pathFidingConfig.hpp"

// CatmullRomSpline
class Spline {

  private:
    // Spline
    std::vector<Point> points;     // Control points after RDP
    std::vector<Point> tangents;   // Tangent vectors at each control point
    std::vector<double> t;         // Parameter values associated with each control point (not time)
    double alpha = constant.alpha; // Exponent used for parameter spacing

    // For sampling
    std::vector<double> arcLength;        // cumulative arc length values
    int arcSamples = constant.arcSamples; // number of samples used to approximate length

  public:
    /// @brief Compute parameter spacing t_i and tangent vectors m_i
    /// @param pts Raw points that need smooth curve
    /// @param start_tangent Start direction (optional)
    /// @param end_tangent End direction (optional)
    Spline(const std::vector<Point>& pts,
                     Point start_tangent = {NAN,NAN}, 
                     Point end_tangent = {NAN,NAN}) :
                     points(pts) {

        int n = points.size();
        t.resize(n,0.0);
        tangents.resize(n);

        // --------------------------------------------
        // STEP 1: Compute parameter values (centripetal)
        //
        // Formula:
        // t_i = t_{i-1} + ||P_i - P_{i-1}||^alpha
        // --------------------------------------------
        for(int i=1;i<n;i++){
            double dx = points[i].x-points[i-1].x;
            double dy = points[i].y-points[i-1].y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist < 1e-6)
                dist = 1e-6;

            t[i] = t[i-1] + pow(dist, alpha);
        }

        // --------------------------------------------
        // STEP 2: Compute tangent vectors
        //
        // Internal points:
        // (P[i+1] - P[i-1]) / (t[i+1] - t[i-1])
        //
        // Endpoints:
        // m_0 = P1 - P0
        // m_n = Pn - P(n-1)
        // --------------------------------------------

        // Compute tangents
        for(int i=1;i<n-1;i++){
            tangents[i].x = (points[i+1].x - points[i-1].x) / (t[i+1] - t[i-1]);
            tangents[i].y = (points[i+1].y - points[i-1].y) / (t[i+1] - t[i-1]);
        }

        // Start tangents (clampled)
        if(std::isnan(start_tangent.x))
        {
            tangents[0].x = (points[1].x - points[0].x) / (t[1] - t[0]);
            tangents[0].y = (points[1].y - points[0].y) / (t[1] - t[0]);
        } else tangents[0] = start_tangent;

        // End tangent (campled)
        if(std::isnan(end_tangent.x))
        {
            tangents[n-1].x = (points[n-1].x - points[n-2].x) / (t[n-1] - t[n-2]);
            tangents[n-1].y = (points[n-1].y - points[n-2].y) / (t[n-1] - t[n-2]);
        } else tangents[n-1] = end_tangent;
    }
    
    double startParameter() const { return t.front(); }

    double endParameter() const { return t.back(); }

    /// @brief
    /// Computes the cubic Hermite interpolation between two control points.
    /// Mathematically, this implements:
    ///
    ///     C(t) =
    ///       h00(τ) * P0
    ///     + h10(τ) * (t1 - t0) * m0
    ///     + h01(τ) * P1
    ///     + h11(τ) * (t1 - t0) * m1
    ///
    /// where:
    ///     τ = (t_query - t0) / (t1 - t0)
    ///
    /// and the Hermite basis functions are:
    ///
    ///     h00(τ) =  2τ³ - 3τ² + 1
    ///     h10(τ) =  τ³ - 2τ² + τ
    ///     h01(τ) = -2τ³ + 3τ²
    ///     h11(τ) =  τ³ - τ²
    ///
    /// These basis functions guarantee:
    ///   - C(t0) = P0
    ///   - C(t1) = P1
    ///   - C'(t0) = m0
    ///   - C'(t1) = m1
    ///
    /// So this segment:
    ///   - Starts at P0
    ///   - Ends at P1
    ///   - Leaves P0 with direction m0
    ///   - Arrives at P1 with direction m1
    ///
    /// @param P0 Starting position of the segment
    /// @param P1 Ending position of the segment
    /// @param m0 Tangent (velocity vector) at P0
    /// @param m1 Tangent (velocity vector) at P1
    /// @param t0 Global parameter value at P0
    /// @param t1 Global parameter value at P1
    /// @param t_query Parameter value where we want to evaluate the curve
    ///
    /// @return 2D position of the spline at t_query
    Point hermite(const Point& P0, const Point& P1, const Point& m0, const Point& m1,
                  double t0, double t1, double t_query);

    /// @brief Public position function
    /// @param t_query Parameter value along the spline
    /// @return 2D position on the smooth curve
    Point position(double t_query);

    /// @brief Velocity at position t
    /// @param t_query Parameter value along the spline
    /// @param dt Small step used for derivative approximation
    /// @return 2D velocity vector
    Point velocity(double t_query, double dt=1e-5);
    
    //////////////////////////////////
    // 
    // SAMPLING FUNCTIONS
    //
    //////////////////////////////////

    /// @brief Build arc-length lookup table
    /// This approximates the curve length by sampling many small segments.
    /// 
    /// We compute:
    ///     s_i = s_{i-1} + || C(t_i) - C(t_{i-1}) ||
    ///
    /// After this, we have:
    ///     arcLength[k]  corresponds to parameter value  t_sample[k]
    ///
    /// @param samples Repetion to build the arc. More sample, smoother curve/path
    void buildArcLengthTable();

    /// @brief Returns total arc length of the spline
    double totalLength() const;

    /// @brief Convert arc length (distance along curve) into parameter t
    /// @param s Desired distance along curve
    /// @return parameter value t such that arc length ≈ s
    double parameterAtArcLength(double s) const;

    /// @brief Generate evenly spaced path points along the spline
    /// @param spacing Desired spacing between points (in inches)
    /// @return Vector of evenly spaced positions
    std::vector<Point> generateEvenPath(double spacing = constant.spacing);

    /// @brief Find the closest point on the path. Can be use in case the robot failed to 
    /// find the next point, we determine what is the next point the robot should go.
    /// @param robot Position of the robot (CHANGE TO POSE)
    /// @param path Points of the path
    /// @return Closest point from the position we are on
    ClosestPointResult findClosestPointOnPath(const Point& robot, const std::vector<Point>& path);

};