#include "pisar/vision/path_extraction.h"

#include <easy/profiler.h>

#include <unordered_set>
#include <queue>

#include <iostream>

namespace pisar::mcp {

/**
 * @brief Class to find the longest path in a set of connected points.
 */
class PathExtractor {
private:
    /**
     * @brief Hash function for Eigen::Vector2i to use in unordered_map.
     */
    struct Vector2iHash
    {
        inline size_t operator()(const Eigen::Vector2i& v) const
        {
            return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y());
        }
    };

    std::unordered_map<Eigen::Vector2i, std::vector<Eigen::Vector2i>, Vector2iHash> m_graph;

public:
    /**
     * @brief Finds the longest path in a set of connected points.
     * @param points Vector of unordered points forming a connected path/maze.
     * @return Ordered vector of points representing the longest path.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> extractLongestPath(const std::vector<Eigen::Vector2i>& points)
    {
        if (points.empty()) return {};

        if (points.size() == 1) return points;

        buildGraph(points);

        // Step 1: Find farthest point from an arbitrary start (first point)
        Eigen::Vector2i firstEndpoint = points[0];
        Eigen::Vector2i farthest1 = bfsFarthest(firstEndpoint).first;

        // Step 2: Find longest path from farthest1
        return getLongestPath(farthest1);
    }

private:

    /**
     * @brief Builds an adjacency list from input points.
     * @param points Set of unordered connected points.
     */
    void buildGraph(const std::vector<Eigen::Vector2i>& points)
    {
        if (points.empty())
        {
            std::cerr << "[ERROR] No points provided to buildGraph()." << std::endl;
            return;
        }

        m_graph.clear();
        std::unordered_set<Eigen::Vector2i, Vector2iHash> pointSet(points.begin(), points.end());

        for (const Eigen::Vector2i& p : points)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if (dx == 0 && dy == 0) continue; // Skip self
                    Eigen::Vector2i neighbor(p.x() + dx, p.y() + dy);
                    if (pointSet.count(neighbor))
                    {
                        addEdge(p, neighbor);
                    }
                }
            }
        }
    }


    /**
     * @brief Adds an undirected edge between two points.
     * @param a First point.
     * @param b Second point.
     */
    void addEdge(const Eigen::Vector2i& a, const Eigen::Vector2i& b)
    {
        m_graph[a].push_back(b);
        m_graph[b].push_back(a);
    }


    /**
     * @brief Runs BFS to find the farthest point from a given start.
     * @param start The starting point.
     * @return A pair containing the farthest point and its distance.
     */
    [[nodiscard]] std::pair<Eigen::Vector2i, int> bfsFarthest(const Eigen::Vector2i& start) const
    {
        std::unordered_map<Eigen::Vector2i, int, Vector2iHash> dist;
        std::queue<Eigen::Vector2i> q;

        q.push(start);
        dist[start] = 0;

        Eigen::Vector2i farthest = start;
        int max_dist = 0;

        while (!q.empty())
        {
            Eigen::Vector2i curr = q.front();
            q.pop();

            for (const Eigen::Vector2i& neighbor : m_graph.at(curr))
            {
                if (dist.find(neighbor) == dist.end()) { // Not visited
                    dist[neighbor] = dist[curr] + 1;
                    q.push(neighbor);
                    if (dist[neighbor] > max_dist) {
                        max_dist = dist[neighbor];
                        farthest = neighbor;
                    }
                }
            }
        }
        return {farthest, max_dist};
    }

    /**
     * @brief Reconstructs the longest path using BFS.
     * @param start The starting point.
     * @return Ordered vector of points forming the longest path.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> getLongestPath(const Eigen::Vector2i& start) const
    {
        std::unordered_map<Eigen::Vector2i, Eigen::Vector2i, Vector2iHash> parent;
        std::queue<Eigen::Vector2i> q;
        std::unordered_map<Eigen::Vector2i, int, Vector2iHash> dist;

        q.push(start);
        dist[start] = 0;
        Eigen::Vector2i farthest = start;

        while (!q.empty()) {
            Eigen::Vector2i curr = q.front();
            q.pop();

            for (const Eigen::Vector2i& neighbor : m_graph.at(curr)) {
                if (dist.find(neighbor) == dist.end()) { // Not visited
                    dist[neighbor] = dist[curr] + 1;
                    parent[neighbor] = curr;
                    q.push(neighbor);
                    farthest = neighbor;
                }
            }
        }

        // Reconstruct path from farthest node back to start
        std::vector<Eigen::Vector2i> longestPath;
        for (Eigen::Vector2i p = farthest; parent.find(p) != parent.end(); p = parent[p]) {
            longestPath.push_back(p);
        }
        longestPath.push_back(start);
        reverse(longestPath.begin(), longestPath.end());

        return longestPath;
    }

};

[[nodiscard]] std::vector<Eigen::Vector2i> extractLongestPath(const std::vector<Eigen::Vector2i>& points)
{
    EASY_FUNCTION();
    return PathExtractor().extractLongestPath(points);
}

}