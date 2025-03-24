
#include "pisar/mcp/vision/homography.h"
#include "pisar/mcp/vision/debug_visualization.h"
#include "pisar/mcp/config.h"

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>

using namespace pisar::mcp;

constexpr bool kDebug = true;

// Pixel points
const auto kImagePoints = std::to_array<Eigen::Vector2i>({
    {404,246}, {970,613}, {1541,607}, {1539,1137}, {1533,1964}, {2009,232}, {2396,591}, {3101,563}
});

// World points (real positions in cm)
const auto kWorldPoints = std::to_array<Eigen::Vector2d>({
    {-9,30}, {-4,25}, {0,25}, {0,20}, {0,15}, {4,30}, {6,25}, {11,25}
});

const cv::Size kTestFrameSize = {3280, 2464};
const float kProjectionTolerance = 0.5f;

const std::string_view kWindowName = "Projection Comparison";

// Adjustable parameters (starting values)
double camera_tilt_deg = -45;
double camera_height_cm = 12;
double camera_offset_cm = 9.4;

void visualizeProjectionComparison(
    const std::span<const Eigen::Vector2d>& expected_world_points,
    const std::span<const Eigen::Vector2d>& projected_world_points,
    double scale = 30.0,
    int padding = 100
) {
    // Find bounding box of all points
    Eigen::Vector2d min_pt = expected_world_points[0];
    Eigen::Vector2d max_pt = expected_world_points[0];

    for (size_t i = 0; i < expected_world_points.size(); ++i) {
        min_pt = min_pt.cwiseMin(expected_world_points[i]).cwiseMin(projected_world_points[i]);
        max_pt = max_pt.cwiseMax(expected_world_points[i]).cwiseMax(projected_world_points[i]);
    }

    // Calculate image size
    int img_width = static_cast<int>((max_pt.x() - min_pt.x()) * scale) + 2 * padding;
    int img_height = static_cast<int>((max_pt.y() - min_pt.y()) * scale) + 2 * padding;

    // Create blank canvas
    cv::Mat canvas(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));

    // Coordinate transform (world to image pixels)
    auto to_canvas = [&](const Eigen::Vector2d& pt) -> cv::Point {
        int x = static_cast<int>((pt.x() - min_pt.x()) * scale) + padding;
        int y = static_cast<int>((max_pt.y() - pt.y()) * scale) + padding; // flip Y for display
        return cv::Point(x, y);
    };

    for (size_t i = 0; i < expected_world_points.size(); ++i) {
        cv::Point expected_pt = to_canvas(expected_world_points[i]);
        cv::Point projected_pt = to_canvas(projected_world_points[i]);

        // Draw expected point (green)
        cv::circle(canvas, expected_pt, 5, cv::Scalar(0, 255, 0), -1);

        // Draw projected point (red)
        cv::circle(canvas, projected_pt, 5, cv::Scalar(0, 0, 255), -1);

        // Draw connecting line (gray)
        const double error = (expected_world_points[i] - projected_world_points[i]).norm();
        cv::Scalar line_color;
        if (error < kProjectionTolerance)
        {
            line_color = cv::Scalar(0, 255, 0);
        }
        else
        {
            line_color = cv::Scalar(0, 0, 255);
        }

        cv::line(canvas, expected_pt, projected_pt, line_color, 1, cv::LINE_AA);
    }

    // Show the result
    cv::imshow(kWindowName.data(), canvas);
}

// Callback function for trackbars
void updateHomography()
{
    std::cout << "Projecting with tilt = " << camera_tilt_deg << " deg, height = " << camera_height_cm << " cm, offset = " << camera_offset_cm << " cm" << std::endl;

    // Adjust tilt angle
    double camera_tilt_rad = pisar::mcp::deg_to_rad<double>(camera_tilt_deg);

    // Create updated camera transform
    CameraTransform updated_camera_transform {
        .position = {0, camera_offset_cm, camera_height_cm}, // Adjust height
        .tilt = Eigen::AngleAxisd(camera_tilt_rad, Eigen::Vector3d::UnitX())
    };

    // Recompute homography
    HomographyProjection projection(updated_camera_transform, gkCameraAxisMapping, gkCameraCalibration);
    HomographySizedProjection sized_projection = projection.for_image(kTestFrameSize);

    // Project all image points
    auto projected_world_points = sized_projection.project(std::span(kImagePoints));

    // Show visualization
    visualizeProjectionComparison(std::span(kWorldPoints), std::span(projected_world_points));
}

void updateTiltAngle(int tilt, void*)
{
    camera_tilt_deg = -tilt;
    updateHomography();
}

void updateCameraHeight(int height_mm, void*)
{
    camera_height_cm = height_mm / 10.0;
    updateHomography();
}

void updateCameraOffset(int offset_mm, void*)
{
    camera_offset_cm = offset_mm / 10.0;
    updateHomography();
}

int main()
{
    // Create OpenCV window
    cv::namedWindow(kWindowName.data(), cv::WINDOW_KEEPRATIO);

    // Create trackbars
    cv::createTrackbar("Tilt (deg)", kWindowName.data(), nullptr, 70, updateTiltAngle);
    cv::setTrackbarMin("Tilt (deg)", kWindowName.data(), 30);
    cv::setTrackbarMax("Tilt (deg)", kWindowName.data(), 70);
    cv::setTrackbarPos("Tilt (deg)", kWindowName.data(), 45);

    cv::createTrackbar("Height (mm)", kWindowName.data(), nullptr, 130, updateCameraHeight);
    cv::setTrackbarMin("Height (mm)", kWindowName.data(), 110);
    cv::setTrackbarMax("Height (mm)", kWindowName.data(), 130);
    cv::setTrackbarPos("Height (mm)", kWindowName.data(), camera_height_cm * 10);

    cv::createTrackbar("Offset (mm)", kWindowName.data(), nullptr, 120, updateCameraOffset);
    cv::setTrackbarMin("Offset (mm)", kWindowName.data(), 80);
    cv::setTrackbarMax("Offset (mm)", kWindowName.data(), 120);
    cv::setTrackbarPos("Offset (mm)", kWindowName.data(), camera_offset_cm * 10);

    // Initial call to display
    updateHomography();

    // Keep updating until user closes the window
    while (cv::waitKey(10) != 27); // Press 'Esc' to exit

    return 0;
}
