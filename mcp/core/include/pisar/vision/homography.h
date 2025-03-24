#pragma once

#include <span>
#include <Eigen/Dense>
#include "pisar/vision/camera.h"

namespace pisar::mcp {

class HomographySizedProjection {
private:
    Eigen::Vector3d m_camera_position;
    Eigen::Matrix3d m_inverse_world_to_image_transformation;

public:
    inline HomographySizedProjection(
        const Eigen::Vector3d cam_position,
        const Eigen::Matrix3d& inverse_world_to_camera_transformation,
        const Eigen::Matrix3d& camera_instrinsic_matrix
    ) :
        m_camera_position(cam_position),
        m_inverse_world_to_image_transformation(inverse_world_to_camera_transformation * camera_instrinsic_matrix.inverse())
    {
    }

    inline HomographySizedProjection(
        const Eigen::Vector3d cam_position,
        const Eigen::Matrix3d& inverse_world_to_camera_transformation,
        const CameraCalibrationData& cam_calib,
        const cv::Size& image_size_px,
        const std::optional<cv::Rect> crop_in_calibrated_image = std::nullopt
    ) : HomographySizedProjection(cam_position, inverse_world_to_camera_transformation, cam_calib.get_transformation(image_size_px, crop_in_calibrated_image))
    {
    }

    inline HomographySizedProjection(
        const CameraTransform& cam_transform,
        const Eigen::Matrix3d& axis_mapping,
        const Eigen::Matrix3d& camera_instrinsic_matrix
    ) :
        m_camera_position(cam_transform.position),
        m_inverse_world_to_image_transformation((
            camera_instrinsic_matrix * axis_mapping * cam_transform.tilt.toRotationMatrix()
        ).inverse())
    {
    }

    inline HomographySizedProjection(
        const CameraTransform cam_transform,
        const Eigen::Matrix3d& axis_mapping,
        const CameraCalibrationData& cam_calib,
        const cv::Size& image_size_px,
        const std::optional<cv::Rect> crop_in_calibrated_image = std::nullopt
    ) : HomographySizedProjection(cam_transform, axis_mapping, cam_calib.get_transformation(image_size_px, crop_in_calibrated_image))
    {
    }

    [[nodiscard]]
    inline Eigen::Vector2d project(const Eigen::Vector2i& pixel_coord) const
    {
        const Eigen::Vector3i pixel_homogeneous = pixel_coord.homogeneous();
        const Eigen::Vector3d world_homogeneous = m_inverse_world_to_image_transformation * pixel_homogeneous.cast<double>();

        const double zc = m_camera_position.z() / world_homogeneous.z();
        return world_homogeneous.head<2>() * zc + m_camera_position.head<2>();
    }

    inline void project(const std::span<const Eigen::Vector2i> pixel_coords, const std::span<Eigen::Vector2d> world_coords) const
    {
        for (int i = 0; i < pixel_coords.size(); ++i)
        {
            world_coords[i] = project(pixel_coords[i]);
        }
    }

    [[nodiscard]] inline std::vector<Eigen::Vector2d> project(const std::span<const Eigen::Vector2i> pixel_coords) const
    {
        std::vector<Eigen::Vector2d> world_coords(pixel_coords.size(), Eigen::Vector2d::Zero());
        project(pixel_coords, std::span(world_coords));
        return world_coords;
    }

    [[nodiscard]] inline Eigen::Vector3d cameraPosition() const
    {
        return m_camera_position;
    }
};

class HomographyProjection {
private:
    Eigen::Vector3d m_camera_position;
    Eigen::Matrix3d m_inverse_world_to_camera_transformation;
    CameraCalibrationData m_cam_calibration;

public:
    inline HomographyProjection(
        const CameraTransform& cam_transform,
        const Eigen::Matrix3d& axis_mapping,
        const CameraCalibrationData cam_calib
    ) :
        m_camera_position(cam_transform.position),
        m_inverse_world_to_camera_transformation((
            axis_mapping * cam_transform.tilt.toRotationMatrix()
        ).inverse()),
        m_cam_calibration(cam_calib)
    {
    }

    [[nodiscard]]
    inline HomographySizedProjection for_image(
        const cv::Size& image_size_px,
        const std::optional<cv::Rect> crop_in_calibrated_image = std::nullopt) const
    {
        return HomographySizedProjection(
            m_camera_position,
            m_inverse_world_to_camera_transformation,
            m_cam_calibration,
            image_size_px,
            crop_in_calibrated_image
        );
    }

};

}