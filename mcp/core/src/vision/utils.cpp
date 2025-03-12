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

}