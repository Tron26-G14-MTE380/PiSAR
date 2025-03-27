#pragma once

#include <Eigen/Dense>
#include <span>

namespace pisar::mcp {
namespace detail {

    template<typename T>
    void simplifyPathImpl(const std::span<const Eigen::Vector2<T>>& points, double tolerance, std::vector<Eigen::Vector2<T>>& output)
    {
        const int end_index = points.size() - 1;

        const Eigen::Vector2<T>& line_start = points[0];
        const Eigen::Vector2<T>& line_end = points[end_index];
        const Eigen::Vector2<T> line_vector = line_end - line_start;
        const double line_vector_length = line_vector.template cast<double>().norm();

        const int line_coeff_a = line_vector.y();
        const int line_coeff_b = -line_vector.x();
        const int line_coeff_c = -(line_coeff_a * line_start.x() + line_coeff_b * line_start.y());

        // Find the point with the maximum distance from line between the start and end
        double max_perpendicular_distance = 0.0;
        int index = 0;
        for (int i = 1; i < end_index; ++i)
        {
            const int center_points_expr = line_coeff_a * points[i].x() + line_coeff_b * points[i].y() + line_coeff_c;
            const double perpendicular_distance = std::abs(center_points_expr) / line_vector_length;

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
}

    /**
     * @brief Simplifies the input path using the Ramer-Douglas-Peucker (RDP) algorithm
     *
     * @param points The points marking the path, must be ordered.
     * @param tolerance Threshold distance that determines how much a point can deviate from a line segment
     * before it is significant enouph to keep without not simplifying. Increasing makes the curves more simple.
     */
    template<typename T>
    [[nodiscard]] std::vector<Eigen::Vector2<T>> simplifyPath(const std::span<const Eigen::Vector2<T>>& points, double tolerance)
    {
        if (points.size() < 3)
        {
            return std::vector<Eigen::Vector2<T>>(points.begin(), points.end());
        }

        std::vector<Eigen::Vector2<T>> output;
        detail::simplifyPathImpl<T>(points, tolerance, output);
        return output;
    }
};