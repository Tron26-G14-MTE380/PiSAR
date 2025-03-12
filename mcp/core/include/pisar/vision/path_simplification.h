#pragma once

#include <Eigen/Dense>
#include <span>

namespace pisar::mcp {
    
/**
     * @brief Simplifies the input path using the Ramer-Douglas-Peucker (RDP) algorithm
     * 
     * @param points The points marking the path, must be ordered.
     * @param tolerance Threshold distance that determines how much a point can deviate from a line segment 
     * before it is significant enouph to keep without not simplifying. Increasing makes the curves more simple.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> simplifyPath(const std::span<const Eigen::Vector2i>& points, double tolerance);
};