#pragma once

#include "pisar/mcp/vision/utils.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <optional>
#include <span>

namespace pisar::mcp {

struct CameraTransform {
    Eigen::Vector3d position;
    Eigen::AngleAxisd tilt;
};

class CameraCaptureConfig {
private:
    cv::Size m_full_size;                         ///< Camera sensor full-resolution
    cv::Size m_binned_size;                       ///< Post-binning resolution (downscale).
    std::optional<cv::Point> m_capture_offset;    ///< Top-left point of the capture crop. std::nullopt if using perfect center crop.
    cv::Size m_capture_size;                      ///< Cropped binned image resolution.
    cv::Size m_downscaled_size;                   ///< post digital downscaling frame size.

    unsigned int m_framerate;

public:

    CameraCaptureConfig(
        const cv::Size& full_size, unsigned int binned_mode, std::optional<cv::Point> capture_offset,
        cv::Size capture_size, unsigned int downscale_factor, unsigned int framerate
    ) : m_full_size(full_size),
        m_binned_size(full_size.width / binned_mode, full_size.height / binned_mode),
        m_capture_offset(capture_offset),
        m_capture_size(capture_size),
        m_downscaled_size(m_capture_size.width / downscale_factor, m_capture_size.height / downscale_factor),
        m_framerate(framerate)
    {
        // TODO: Validation
    }

    [[nodiscard]] inline const cv::Size& fullSize() const { return m_full_size; }
    [[nodiscard]] inline const cv::Size& binnedSize() const { return m_binned_size; }
    [[nodiscard]] inline const cv::Size& captureOffset() const { return m_capture_offset.value_or(cv::Point(0, 0)); }
    [[nodiscard]] inline const cv::Size& captureSize() const { return m_capture_size; }
    [[nodiscard]] inline const cv::Size& downscaledSize() const { return m_downscaled_size; }
    [[nodiscard]] inline unsigned int framerate() const { return m_framerate; }

    /// @brief Return the crop region of the image captured from the binned output.
    [[nodiscard]] inline cv::Rect captureCrop() const
    {
        if (m_capture_offset)
        {
            return cv::Rect(m_capture_offset.value().x, m_capture_offset.value().y, m_capture_size.width, m_capture_size.height);
        }

        return computeCenterCrop(m_binned_size, m_capture_size);
    }
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
     * @param capture camera capture configuration.
     * @return A intrinsic transformation adjusted for the new crop and scale.
     */
    [[nodiscard]]
    inline Eigen::Matrix3d get_transformation(const CameraCaptureConfig& capture) const
    {
        // Ensure calibration is at full resolution
        assert(capture.fullSize().width == calibration_img_size_px.x() &&
        capture.fullSize().height == calibration_img_size_px.y() &&
        "CameraCalibrationData must be calibrated at full sensor resolution");

        // Get the crop in the binned image (this is in binned coordinates)
        const cv::Rect binned_crop = capture.captureCrop();

        // Scale crop rect to calibration (full-res) coordinates
        const double scale_x = static_cast<double>(capture.fullSize().width) / capture.binnedSize().width;
        const double scale_y = static_cast<double>(capture.fullSize().height) / capture.binnedSize().height;

        const cv::Rect crop_in_calibrated = {
            static_cast<int>(std::round(binned_crop.x * scale_x)),
            static_cast<int>(std::round(binned_crop.y * scale_y)),
            static_cast<int>(std::round(binned_crop.width * scale_x)),
            static_cast<int>(std::round(binned_crop.height * scale_y))
        };

        // Compute scale from crop to downscaled output
        const Eigen::Vector2d scale = Eigen::Vector2d(
            capture.downscaledSize().width,
            capture.downscaledSize().height
        ).cwiseQuotient(Eigen::Vector2d(crop_in_calibrated.width, crop_in_calibrated.height));

        // Adjust focal length
        const Eigen::Vector2d scaled_focal_length = focal_length_px.cwiseProduct(scale);

        // Adjust principal point
        const Eigen::Vector2d adjusted_principal_point =
        (principle_axis_offset_px - Eigen::Vector2d(crop_in_calibrated.x, crop_in_calibrated.y)).cwiseProduct(scale);

        return Eigen::Matrix3d{
            {scaled_focal_length.x(), skew, adjusted_principal_point.x()},
            {0, scaled_focal_length.y(), adjusted_principal_point.y()},
            {0, 0, 1}
        };
    }


    [[nodiscard]]
    inline Eigen::Matrix3d get_inverse_transformation(const CameraCaptureConfig& capture) const
    {
        return get_transformation(capture).inverse();
    }

    /**
     * @brief Undistorts a distorted pixel coordinate using this camera's distortion model.
     * @param distorted_px The distorted pixel coordinate (u, v).
     * @param capture camera capture configuration.
     * @param iterations Number of refinement iterations (default: 5).
     * @return Undistorted pixel coordinate (u', v') in the same image space.
     */
    [[nodiscard]]
    inline Eigen::Vector2i undistortPixel(
        const Eigen::Vector2i& distorted_px,
        const CameraCaptureConfig& capture,
        int iterations = 5
    ) const
    {
        Eigen::Matrix3d intrinsic_transformation = get_transformation(capture);
        return undistortPixel(distorted_px, intrinsic_transformation, iterations);
    }

    /**
     * @brief Undistorts a span of distorted pixel coordinates using this camera's distortion model.
     *
     * @param distorted_pixels Span of distorted pixel coordinates.
     * @param capture camera capture configuration.
     * @param iterations Number of refinement iterations (default: 5).
     * @return A vector of undistorted pixel coordinates (u', v').
     */
    [[nodiscard]]
    inline std::vector<Eigen::Vector2i> undistortPixel(
        const std::span<const Eigen::Vector2i> distorted_pixels,
        const CameraCaptureConfig& capture,
        int iterations = 5
    ) const
    {
        Eigen::Matrix3d intrinsic_transformation = get_transformation(capture);

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