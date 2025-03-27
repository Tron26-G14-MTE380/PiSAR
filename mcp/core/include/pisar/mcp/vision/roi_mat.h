#pragma once

#include <opencv2/opencv.hpp>

#include <optional>

/**
 * @brief Wrapper around cv::Mat that tracks cropping offset and original image size.
 */
class RoiMat {
private:
cv::Mat m_image;           ///< The OpenCV image
cv::Point m_offset;        ///< Offset from the original image
cv::Size m_original_size;  ///< Size of the original image

public:
    /**
     * @brief Construct an empty RoiMat.
     */
    RoiMat() = default;

    /**
     * @brief Construct an RoiMat with an image.
     * @param image The OpenCV image.
     */
    explicit RoiMat(
        const cv::Mat& image,
        const RoiMat& original_image
    ) :
        m_image(image.clone()),
        m_offset(original_image.getOffset()),
        m_original_size(original_image.getOriginalSize()) {}

    explicit RoiMat(
        const cv::Mat& image,
        std::optional<cv::Point> offset = std::nullopt,
        std::optional<cv::Size> original_size = std::nullopt
    ) :
        m_image(image.clone()),
        m_offset(offset ? offset.value() : cv::Point{0, 0}),
        m_original_size(original_size.has_value() ? original_size.value() : image.size()) {}

    /**
     * @brief Get the underlying cv::Mat image.
     * @return Reference to the stored image.
     */
    [[nodiscard]] cv::Mat& getMat() { return m_image; }

    /**
     * @brief Get a constant reference to the stored image.
     * @return Constant reference to the stored image.
     */
    [[nodiscard]] const cv::Mat& getMat() const { return m_image; }

    /**
     * @brief Get the offset of the current image relative to the original.
     * @return The cropping offset as cv::Point.
     */
    [[nodiscard]] cv::Point getOffset() const { return m_offset; }

    /**
     * @brief Get the original image size before cropping.
     * @return The original image size as cv::Size.
     */
    [[nodiscard]] cv::Size getOriginalSize() const { return m_original_size; }

    [[nodiscard]] cv::Rect getCrop() const { return cv::Rect(m_offset.x, m_offset.y, m_image.cols, m_image.rows); }

    /**
     * @brief Crop the image and update the offset.
     * @param rect The cropping rectangle.
     * @return Cropped RoiMat object.
     */
    [[nodiscard]] RoiMat crop(const cv::Rect& rect) const
    {
        const cv::Point new_offset = cv::Point(rect.x, rect.y) + this->m_offset;
        return RoiMat(m_image(rect).clone(), new_offset, this->m_original_size);
    }

    /**
     * @brief Convert a point in the cropped image back to original coordinates.
     * @param point The point in the cropped image.
     * @return The corresponding point in the original image.
     */
    [[nodiscard]] cv::Point toOriginalCoords(const cv::Point& point) const
    {
        return point + m_offset;
    }

    /**
     * @brief Restores the cropped region onto a blank image of the original size.
     * @return A cv::Mat of the original image size with the cropped region in place.
     */
    [[nodiscard]] cv::Mat restoreToOriginalSize() const
    {
        if (m_image.empty() || m_original_size == cv::Size(0, 0))
        {
            return cv::Mat {};
        }

        cv::Mat restored_image = cv::Mat::zeros(m_original_size, m_image.type());

        // Ensure the ROI is within bounds
        int roi_x = std::max(0, m_offset.x);
        int roi_y = std::max(0, m_offset.y);
        int roi_width = std::min(m_image.cols, m_original_size.width - roi_x);
        int roi_height = std::min(m_image.rows, m_original_size.height - roi_y);

        if (roi_width > 0 && roi_height > 0) {
            cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
            m_image(cv::Rect(0, 0, roi_width, roi_height)).copyTo(restored_image(roi));
        }

        return restored_image;
    }
};
