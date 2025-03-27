#pragma once

#include <opencv2/opencv.hpp>

namespace pisar::mcp {

/**
 * @brief Detects if the target is in frame through a series of tests.
 *
 */
class TargetDetector {

public:
    /**
     * @brief Scan the current frame and returns if target is in frame.
     *
     * @note It may take multiple frames with the target detected to gain confidence to return true.
     *
     * @param input_frame The frame to scan
     * @return true if the target was found otherwise false.
     */
    [[nodiscard]] bool scanFrame(const cv::Mat& input_frame)
    {

    }

    /**
     * @brief Resets the detector state.
     *
     */
    void reset()
    {

    }

private:
};

}