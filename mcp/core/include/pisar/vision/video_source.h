#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <span>
#include <optional>

#include <opencv2/opencv.hpp>

namespace pisar::mcp {

/**
 * @brief CRTP base class for video sources.
 * @tparam TDerived The derived class implementing the interface.
 */
template <typename TDerived>
class VideoSource {
public:
    /**
     * @brief Constructs a VideoSource with a specified resolution.
     */
    inline VideoSource() noexcept {}

    ~VideoSource() {
        stop();
    }

    /**
     * @brief Starts the video source.
     * @param frame_size Frame size.
     */
    inline void start(const cv::Size frame_size) { static_cast<TDerived*>(this)->startImpl(frame_size); }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame is successful.
     * @retval std::nullopt if no frame captured.
     */
    [[nodiscard]] inline std::optional<cv::Mat> getFrame() { return static_cast<TDerived*>(this)->getFrameImpl(); }

    /**
     * @brief Stops the video source.
     */
    inline void stop() { static_cast<TDerived*>(this)->stopImpl(); }
};

/**
 * @brief Video source that loads a static image and repeatedly returns it.
 */
class RepeatedImageFileSource : public VideoSource<RepeatedImageFileSource> {
private:
    std::string m_filePath;
    cv::Mat m_img;

public:
    /**
     * @brief Constructs a RepeatedImageFileSource.
     * @param file_path Path to the image file.
     * @param width Image width.
     * @param height Image height.
     */
    inline RepeatedImageFileSource(const std::string& file_path)
        : VideoSource(), m_filePath(file_path) {}

    /**
     * @brief Starts the image source by loading the file.
     * @throws std::runtime_error if the image fails to load.
     */
    inline void startImpl(const cv::Size frame_size) 
    {
        if (!m_img.empty()) 
        {
            throw std::runtime_error("Already started");
        }

        m_img = cv::imread(m_filePath);
        if (m_img.empty()) 
        {
            throw std::runtime_error("Failed to load image: " + m_filePath);
        }
        cv::Mat resized;
        cv::resize(m_img, resized, frame_size);
        m_img = resized;
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame is successful.
     * @retval std::nullopt if no frame captured.
     */
    [[nodiscard]] inline std::optional<cv::Mat> getFrameImpl() 
    {
        if (m_img.empty()) {
            throw std::runtime_error("Image not loaded. Call start() first.");
        }
        return m_img.clone();
    }

    /**
     * @brief Stops the image source.
     */
    inline void stopImpl() { m_img.release(); }
};

/**
 * @brief Video source from a camera feed. 
 */
class VideoCameraSource : public VideoSource<VideoCameraSource> {
private:
    int m_camera_id;
    cv::VideoCapture m_cap;

public:
    /**
     * @brief Constructs a VideoCameraSource.
     * @param width Image width.
     * @param height Image height.
     */
    inline VideoCameraSource(int camera_id) : VideoSource(), m_camera_id(camera_id), m_cap() {}

    /**
     * @brief Starts the video capture.
     * @throws std::runtime_error if the camera fails to open.
     */
    inline void startImpl(const cv::Size frame_size) 
    {
        if (m_cap.isOpened()) 
        {
            throw std::runtime_error("Camera already opened");
        }

        if (!m_cap.open(m_camera_id)) 
        {
            throw std::runtime_error("Failed to open camera");
        }

        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame is successful.
     * @retval std::nullopt if no frame captured.
     * @throws std::runtime_error on error.
     */
    [[nodiscard]] inline std::optional<cv::Mat> getFrameImpl() 
    {
        if (!m_cap.isOpened()) 
        {
            throw std::runtime_error("Camera is not started");
        }

        cv::Mat frame;
        while (!m_cap.read(frame)) 
        {
            return std::nullopt;
        }

        return frame;
    }

    /**
     * @brief Stops the video capture and releases the camera.
     */
    inline void stopImpl() 
    {
        if (m_cap.isOpened()) 
        {
            m_cap.release();
        }
    }
};

}