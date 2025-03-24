#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <optional>

namespace pisar::mcp {

struct CameraTransform {
    Eigen::Vector3d position;
    Eigen::AngleAxisd tilt;
};

struct CameraDistortionData
{
    double k1, k2, p1, p2, k3;
};

struct CameraCalibrationData
{
    Eigen::Vector2i calibration_img_size_px;
    Eigen::Vector2d focal_length_px;
    Eigen::Vector2d principle_axis_offset_px;
    double skew;
    CameraDistortionData distortion_coeffs;

    /**
     * @brief Returns a instrinsic matrix for a cropped and rescaled image.
     *
     * If you calibrated at full resolution and are now capturing a cropped portion of the sensor
     * which is then downscaled to a smaller output resolution, this adjusts focal length and
     * principal point accordingly.
     *
     * @param output_image_size_px The final resolution after scaling (e.g., 320x240).
     * @param crop_in_calibrated_image The crop region in full-res calibration image coordinates (e.g., {x, y, w, h}).
     * @return A intrinsic transformation adjusted for the new crop and scale.
     */
    [[nodiscard]]
    inline Eigen::Matrix3d get_transformation(const cv::Size& output_image_size_px,
        const std::optional<cv::Rect> crop_in_calibrated_image = std::nullopt) const
    {
        // By default, assume full-frame calibration
        cv::Rect crop_rect(0, 0, calibration_img_size_px.x(), calibration_img_size_px.y());

        if (crop_in_calibrated_image.has_value())
        {
            crop_rect = crop_in_calibrated_image.value();
        }

        // Compute scale from cropped region to output
        const Eigen::Vector2d scale = Eigen::Vector2d(output_image_size_px.width, output_image_size_px.height).cwiseQuotient(
            Eigen::Vector2d(crop_rect.width, crop_rect.height)
        );

        // Adjust focal length by scaling
        const Eigen::Vector2d scaled_focal_length = focal_length_px.cwiseProduct(scale);

        // Adjust principal point by offsetting crop and scaling
        const Eigen::Vector2d adjusted_principal_point =
        (principle_axis_offset_px - Eigen::Vector2d(crop_rect.x, crop_rect.y)).cwiseProduct(scale);

        return Eigen::Matrix3d {
            {scaled_focal_length.x(), skew, adjusted_principal_point.x()},
            {0, scaled_focal_length.y(), adjusted_principal_point.y()},
            {0, 0, 1}
        };
    }


    [[nodiscard]]
    inline Eigen::Matrix3d get_inverse_transformation(const cv::Size& output_image_size_px,
        const std::optional<cv::Rect> crop_in_calibrated_image = std::nullopt) const
    {
        return get_transformation(output_image_size_px, crop_in_calibrated_image).inverse();
    }

    /**
     * @brief Undistorts a distorted pixel coordinate using this camera's distortion model.
     * @param distorted_px The distorted pixel coordinate (u, v).
     * @param image_size_px The image resolution the point comes from.
     * @param iterations Number of refinement iterations (default: 5).
     * @return Undistorted pixel coordinate (u', v') in the same image space.
     */
    [[nodiscard]]
    inline Eigen::Vector2i undistortPixel(
        const Eigen::Vector2i& distorted_px,
        const cv::Size& image_size_px,
        int iterations = 5
    ) const
    {
        Eigen::Matrix3d intrinsic_transformation = get_transformation(image_size_px);
        return undistortPixel(distorted_px, intrinsic_transformation, iterations);
    }

    /**
     * @brief Undistorts a span of distorted pixel coordinates using this camera's distortion model.
     *
     * @param distorted_pixels Span of distorted pixel coordinates.
     * @param image_size_px The resolution of the image the points come from.
     * @param iterations Number of refinement iterations (default: 5).
     * @return A vector of undistorted pixel coordinates (u', v').
     */
    [[nodiscard]]
    inline std::vector<Eigen::Vector2i> undistortPixel(
        const std::span<const Eigen::Vector2i> distorted_pixels,
        const cv::Size& image_size_px,
        int iterations = 5
    ) const
    {
        Eigen::Matrix3d intrinsic_transformation = get_transformation(image_size_px);

        std::vector<Eigen::Vector2i> result;
        result.reserve(distorted_pixels.size());

        for (const auto& px : distorted_pixels)
        {
            result.push_back(undistortPixel(px, intrinsic_transformation, iterations));
        }

        return result;
    }

private:
    /**
     * @brief Undistorts a distorted pixel coordinate using this camera's distortion model.
     * @param distorted_px The distorted pixel coordinate (u, v).
     * @param intrinsic_transformation The intrinsic transformation.
     * @param iterations Number of refinement iterations (default: 5).
     * @return Undistorted pixel coordinate (u', v') in the same image space.
     */
    [[nodiscard]]
    inline Eigen::Vector2i undistortPixel(
        const Eigen::Vector2i& distorted_px,
        const Eigen::Matrix3d& intrinsic_transformation,
        int iterations = 5
    ) const
    {
        const double fx = intrinsic_transformation(0, 0);
        const double fy = intrinsic_transformation(1, 1);
        const double cx = intrinsic_transformation(0, 2);
        const double cy = intrinsic_transformation(1, 2);

        // Convert to normalized coordinates
        double x = (distorted_px.x() - cx) / fx;
        double y = (distorted_px.y() - cy) / fy;

        double x_u = x;
        double y_u = y;

        const double k1 = distortion_coeffs.k1;
        const double k2 = distortion_coeffs.k2;
        const double p1 = distortion_coeffs.p1;
        const double p2 = distortion_coeffs.p2;
        const double k3 = distortion_coeffs.k3;

        for (int i = 0; i < iterations; ++i) {
            double r2 = x_u * x_u + y_u * y_u;
            double r4 = r2 * r2;
            double r6 = r4 * r2;

            double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
            double dx = 2.0 * p1 * x_u * y_u + p2 * (r2 + 2.0 * x_u * x_u);
            double dy = 2.0 * p2 * x_u * y_u + p1 * (r2 + 2.0 * y_u * y_u);

            double x_proj = x_u * radial + dx;
            double y_proj = y_u * radial + dy;

            double ex = x - x_proj;
            double ey = y - y_proj;

            x_u += ex;
            y_u += ey;
        }

        // Back to pixel space
        double u_undistorted = fx * x_u + cx;
        double v_undistorted = fy * y_u + cy;

        return Eigen::Vector2i(u_undistorted, v_undistorted);
    }
};


}