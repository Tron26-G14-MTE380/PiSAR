#pragma once

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

namespace pisar::mcp {


/**
 * @brief Finds the longest path in a set of connected points.
 * @param points Vector of unordered points forming a connected path/maze.
 * @return Ordered vector of points representing the longest path.
 */
[[nodiscard]] std::vector<Eigen::Vector2i> extractLongestPath(const std::vector<Eigen::Vector2i>& points);

}