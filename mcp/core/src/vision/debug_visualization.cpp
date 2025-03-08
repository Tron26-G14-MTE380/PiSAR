#include "pisar/vision/debug_visualization.h"

namespace pisar::mcp {

cv::Mat createHomographyProjectionVisualization(
    cv::Size2i image_size,
    const HomographySizedProjection& projection,
    const std::vector<Eigen::Vector2d>& trajectory
)
{
    // Create a blank image
    cv::Mat visualization = cv::Mat::zeros(image_size.height, image_size.width, CV_8UC3);

    // Step 1: Generate a grid of points for visualization
    std::vector<Eigen::Vector2i> camera_view_grid_image_points;
    for (int i = 0; i < 10; ++i)
    {
        for (int j = 0; j < 10; ++j)
        {
            double u = i * (image_size.width / 9.0);
            double v = j * (image_size.height / 9.0);
            camera_view_grid_image_points.emplace_back(static_cast<int>(u), static_cast<int>(v));
        }
    }

    // Project the grid into world space
    std::vector<Eigen::Vector2d> camera_view_grid_world_points = projection.project(std::span(camera_view_grid_image_points));

    // Combine trajectory and projected grid points for world bounds computation
    std::vector<Eigen::Vector2d> all_world_points = trajectory;
    all_world_points.insert(
        all_world_points.end(),
        camera_view_grid_world_points.begin(),
        camera_view_grid_world_points.end()
    );

    // Find world bounds
    double X_min = std::numeric_limits<double>::max(), X_max = std::numeric_limits<double>::lowest();
    double Y_min = std::numeric_limits<double>::max(), Y_max = std::numeric_limits<double>::lowest();

    for (const auto& pt : all_world_points) {
        X_min = std::min(X_min, pt.x());
        X_max = std::max(X_max, pt.x());
        Y_min = std::min(Y_min, pt.y());
        Y_max = std::max(Y_max, pt.y());
    }

    // Compute scale factors
    const double world_x_range = X_max - X_min;
    const double world_y_range = Y_max - Y_min;
    const double world_to_image_scale_factor_x = image_size.width / world_x_range;
    const double world_to_image_scale_factor_y = image_size.height / world_y_range;
    const double world_to_image_scale_factor = std::min(world_to_image_scale_factor_x, world_to_image_scale_factor_y);

    // Function to convert world points to image space
    auto worldToImageFixed = [&](Eigen::Vector2d world_point) -> cv::Point2f {
        float x_scaled = static_cast<float>(world_point.x() * world_to_image_scale_factor + image_size.width / 2);
        float y_scaled = static_cast<float>(image_size.height - (world_point.y() * world_to_image_scale_factor));
        return {x_scaled, y_scaled};
    };

    // Convert points to image space
    std::vector<cv::Point2f> image_grid_points(camera_view_grid_world_points.size(), {0});
    std::transform(
        camera_view_grid_world_points.begin(),
        camera_view_grid_world_points.end(),
        image_grid_points.begin(),
        worldToImageFixed
    );

    std::vector<cv::Point2f> image_trajectory(trajectory.size(), {0});
    std::transform(trajectory.begin(), trajectory.end(), image_trajectory.begin(), worldToImageFixed);

    const Eigen::Vector2d camera_position = projection.cameraPosition().head<2>();
    cv::Point2f image_camera_position = worldToImageFixed(camera_position);

    const Eigen::Vector2d robot_position = Eigen::Vector2d::Zero();
    cv::Point2f image_robot_position = worldToImageFixed(robot_position);

    // Draw grid points
    for (const auto& pt : image_grid_points) {
        cv::circle(visualization, pt, 1, cv::Scalar(255, 255, 255), -1);
    }

    // Draw trajectory
    for (size_t i = 0; i < image_trajectory.size(); ++i)
    {
        cv::circle(visualization, image_trajectory[i], 3, cv::Scalar(0, 0, 255), -1); // Red points
        if (i > 0)
        {
            cv::line(visualization, image_trajectory[i - 1], image_trajectory[i], cv::Scalar(0, 0, 255), 2);
        }
    }

    // Draw Camera
    cv::circle(visualization, image_camera_position, 3, cv::Scalar(0, 255, 0), -1); // Green

    // Draw robot
    cv::circle(visualization, image_robot_position, 3, cv::Scalar(255, 0, 0), -1); // Blue

    return visualization;
}


cv::Mat createDebugCanvas(const std::vector<std::pair<std::string, cv::Mat>>& debug_images, int max_size, int grid_padding)
{
    if (debug_images.empty()) {
        return cv::Mat::ones(100, 100, CV_8UC3) * 255; // Return a blank white canvas if empty
    }

    // Determine minimum width and height
    int min_width = std::numeric_limits<int>::max();
    int min_height = std::numeric_limits<int>::max();

    for (const auto& [key, img] : debug_images)
    {
        min_width = std::min(min_width, img.size().width);
        min_height = std::min(min_height, img.size().height);
    }

    // Ensures no image exceeds 320px in width/height
    double scale_factor = (std::max(min_width, min_height) > max_size) ?
                          static_cast<double>(max_size) / static_cast<double>(std::max(min_width, min_height)) : 1.0;

    // Resize images while maintaining aspect ratio
    std::vector<cv::Mat> resized_images(debug_images.size());
    std::transform(debug_images.begin(), debug_images.end(), resized_images.begin(),
                   [scale_factor](const auto& pair) {
                       cv::Mat resized;
                       cv::resize(pair.second, resized, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
                       return resized;
                   });

    // Convert grayscale to BGR for consistency
    for (auto& img : resized_images)
    {
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        }
    }

    // Determine grid layout
    const int num_images = resized_images.size();
    const int grid_cols = 4;  // Number of columns in the grid
    const int grid_rows = std::ceil(static_cast<double>(num_images) / grid_cols); // Correct floating-point division

    const cv::Size img_size = resized_images[0].size();

    // Create an empty canvas for the grid with padding
    const cv::Size grid_size = {
        (img_size.width + grid_padding) * grid_cols - grid_padding,
        (img_size.height + grid_padding) * grid_rows - grid_padding
    };

    cv::Mat grid_canvas = cv::Mat::ones(grid_size, CV_8UC3) * 255;

    // Place images in the grid with padding
    for (int i = 0; i < resized_images.size(); ++i)
    {
        const int row = i / grid_cols;
        const int col = i % grid_cols;

        cv::Point start_point = {
            col * (img_size.width + grid_padding),
            row * (img_size.height + grid_padding),
        };

        resized_images[i].copyTo(
            grid_canvas(cv::Rect(start_point.x, start_point.y, resized_images[i].cols, resized_images[i].rows))
        );

        // Add text labels
        cv::putText(
            grid_canvas, debug_images[i].first, {start_point.x + 10, start_point.y + 20},
            cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1, cv::LINE_AA
        );
    }

    return grid_canvas;
}

void displayDebug(const cv::Mat debug_canvas)
{
    // Show the composite debug display
    cv::imshow("Debug Data", debug_canvas);
    const int key = cv::waitKey(1);  // Small delay to allow window to refresh

    if (key == 27 || cv::getWindowProperty("Debug Data", cv::WND_PROP_VISIBLE) <= 0)  // ESC key or window closed
    {
        std::cout << "Debug window closed. Exiting program." << std::endl;
        cv::destroyAllWindows();
        std::terminate();
    }
}

}