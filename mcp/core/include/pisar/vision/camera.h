#pragma once

#include <Eigen/Dense>

namespace pisar::mcp {

struct CameraCalibrationData
{
    Eigen::Vector2i calibration_img_size_px;
    Eigen::Vector2i focal_length_px;
    Eigen::Vector2i principle_axis_offset_px;
    double skew;

    [[nodiscard]]
    inline Eigen::Matrix3d get_transformation(const Eigen::Vector2i &image_size_px) const
    {
        const Eigen::Vector2d scale = image_size_px.cast<double>().cwiseQuotient(calibration_img_size_px.cast<double>());
    
        const Eigen::Matrix3d scale_matrix {
            {scale.x(), 0, 0},
            {0, scale.y(), 0},
            {0, 0, 1}
        };
    
        const Eigen::Matrix3d intrinsic_matrix {
            {static_cast<double>(focal_length_px.x()), skew, static_cast<double>(principle_axis_offset_px.x())},
            {0, static_cast<double>(focal_length_px.y()), static_cast<double>(principle_axis_offset_px.y())},
            {0, 0, 1}
        };
    
        return scale_matrix * intrinsic_matrix;
    }
    
    [[nodiscard]]
    inline Eigen::Matrix3d get_inverse_transformation(const Eigen::Vector2i &image_size_px) const
    {
        return get_transformation(image_size_px).inverse();
    }
};


}