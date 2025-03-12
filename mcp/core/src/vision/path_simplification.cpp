#include "pisar/vision/path_simplification.h"

#include <easy/profiler.h>

namespace pisar::mcp {

static void simplifyPathImpl(const std::span<const Eigen::Vector2i>& points, double tolerance, std::vector<Eigen::Vector2i>& output) 
{
    const int end_index = points.size() - 1;

    const Eigen::Vector2i& line_start = points[0];
    const Eigen::Vector2i& line_end = points[end_index];
    const Eigen::Vector2i line_vector = line_end - line_start;
    const double line_vector_length = line_vector.norm();

    const int line_coeff_a = line_vector.y();
    const int line_coeff_b = -line_vector.x();
    const int line_coeff_c = -(line_coeff_a * line_start.x() + line_coeff_b * line_start.y());

    // Find the point with the maximum distance from line between the start and end
    double max_perpendicular_distance = 0.0;
    int index = 0;
    for (int i = 1; i < end_index; ++i) 
    {
        const int center_points_expr = line_coeff_a * points[i].x() + line_coeff_b * points[i].y();
        const double perpendicular_distance = std::abs(line_coeff_a * points[i].x() + line_coeff_b * points[i].y() + line_coeff_c) / line_vector_length;

        if (perpendicular_distance > max_perpendicular_distance) 
        {
            index = i;
            max_perpendicular_distance = perpendicular_distance;
        }
    }

    // If max distance is greater than tolerance, recursively simplify
    if (max_perpendicular_distance > tolerance) 
    {
        simplifyPathImpl(points.subspan(0, index + 1), tolerance, output);
        simplifyPathImpl(points.subspan(index+1, points.size()-index-1), tolerance, output);
    }
    else 
    {
        // Just return start and end points
        output.push_back(line_start);
        output.push_back(line_end);
    }
}


[[nodiscard]]  std::vector<Eigen::Vector2i> simplifyPath(const std::span<const Eigen::Vector2i>& points, double tolerance) 
{
    EASY_FUNCTION();

    if (points.size() < 3) 
    {
        return std::vector<Eigen::Vector2i>(points.begin(), points.end());
    }

    std::vector<Eigen::Vector2i> output;
    simplifyPathImpl(points, tolerance, output);
    return output;
}

}