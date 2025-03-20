#include "pisar/vision/utils.h"

namespace pisar::mcp {

cv::Rect computeBoundingBox(const cv::Mat& img) {
    CV_Assert(img.type() == CV_8UC1);

    int min_x = 0, max_x = img.cols - 1, min_y = 0, max_y = img.rows - 1;

    // Find first and last rows that contain nonzero pixels
    while (min_y <= max_y && cv::countNonZero(img.row(min_y)) == 0) ++min_y;
    while (max_y >= min_y && cv::countNonZero(img.row(max_y)) == 0) --max_y;

    if (min_y > max_y) return cv::Rect(); // No nonzero pixels found

    //return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);

    // Find first and last columns that contain nonzero pixels
    cv::Mat roi = img.rowRange(min_y, max_y + 1);
    while (min_x < img.cols && cv::countNonZero(roi.col(min_x)) == 0) ++min_x;
    while (max_x < img.cols && cv::countNonZero(roi.col(max_x)) == 0) --max_x;

    return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

cv::Mat resizeWithPadding(const cv::Mat& input, const cv::Size& target_size)
{
    if (input.empty())
    {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return cv::Mat();
    }

    const int original_width = input.cols;
    const int original_height = input.rows;
    const double aspect_ratio = static_cast<double>(original_width) / original_height;

    const int target_width = target_size.width;
    const int target_height = target_size.height;
    const double target_aspect_ratio = static_cast<double>(target_width) / target_height;

    int new_width, new_height;

    // Resize while keeping aspect ratio
    if (aspect_ratio > target_aspect_ratio)
    {
        new_width = target_width;
        new_height = static_cast<int>(target_width / aspect_ratio);
    }
    else
    {
        new_height = target_height;
        new_width = static_cast<int>(target_height * aspect_ratio);
    }

    cv::Mat resized;
    cv::resize(input, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

    // Compute padding
    int top = (target_height - new_height) / 2;
    int bottom = target_height - new_height - top;
    int left = (target_width - new_width) / 2;
    int right = target_width - new_width - left;

    // Create output image with padding
    cv::Mat output;
    cv::copyMakeBorder(resized, output, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); // Black padding

    return output;
}

cv::Mat downscaleCrop(const cv::Mat &input, const cv::Size &target_size)
{
    if (input.empty())
    {
        throw std::runtime_error("Input image is empty");
    }

    int original_width = input.cols;
    int original_height = input.rows;

    // Compute scale factors for width and height
    double scale_x = static_cast<double>(target_size.width) / original_width;
    double scale_y = static_cast<double>(target_size.height) / original_height;

    // ✅ Choose the **larger** scaling factor to **fill the entire target_size**
    double scale_factor = std::max(scale_x, scale_y);

    // Compute the new scaled dimensions
    cv::Size new_size(static_cast<int>(original_width * scale_factor),
                      static_cast<int>(original_height * scale_factor));

    // Resize the image with the new scaled size
    cv::Mat scaled_image;
    cv::resize(input, scaled_image, new_size, 0, 0, cv::INTER_AREA);

    // ✅ Crop the center region to match the exact target size
    int crop_x = (new_size.width - target_size.width) / 2;
    int crop_y = (new_size.height - target_size.height) / 2;

    cv::Rect crop_region(crop_x, crop_y, target_size.width, target_size.height);
    cv::Mat cropped_output = scaled_image(crop_region);

    return cropped_output;
}

}
