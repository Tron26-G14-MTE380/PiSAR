#pragma once

#include "pisar/vision/utils.h"
#include "pisar/concurrent_queue.h"

#include <opencv2/opencv.hpp>

#ifdef __linux__
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer_allocator.h>
#endif

#include <memory>
#include <stdexcept>
#include <string>
#include <span>
#include <optional>
#include <filesystem>
#include <iostream>
#include <string_view>

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
class CvCameraVideoSource : public VideoSource<CvCameraVideoSource> {
private:
    int m_camera_id;
    cv::VideoCapture m_cap;

public:
    /**
     * @brief Constructs a CvCameraVideoSource.
     * @param width Image width.
     * @param height Image height.
     */
    inline CvCameraVideoSource(int camera_id) : VideoSource(), m_camera_id(camera_id), m_cap() {}

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

/**
 * @brief Video source using a camera feed (via GStreamer).
 */
class GStreamerCameraVideoSource : public VideoSource<GStreamerCameraVideoSource> {
private:
    int m_camera_id;        ///< Camera ID (ignored for libcamera, but kept for compatibility).
    cv::VideoCapture m_cap; ///< OpenCV VideoCapture object using GStreamer.

public:
    /**
     * @brief Constructs a GStreamerCameraVideoSource.
     * @param camera_id Camera index.
     */
    inline explicit GStreamerCameraVideoSource(int camera_id)
        : VideoSource(), m_camera_id(camera_id), m_cap() {}

    /**
     * @brief Starts the video capture.
     * @param frame_size Image frame size.
     * @throws std::runtime_error if the camera fails to open.
     */
    inline void startImpl(const cv::Size frame_size)
    {
        if (m_cap.isOpened()) {
            throw std::runtime_error("Camera already started.");
        }

        // GStreamer pipeline for libcamera
        std::string p_line1 = "libcamerasrc camera-name=/base/axi/pcie@120000/rp1/i2c@88000/imx219@10 contrast=1.2 ";  // Enable auto-exposure, gain, and denoising

        std::string p_line2 = "! video/x-raw, width=640, height=480, framerate=60/1, format=NV12 ";  // Use NV12 for better quality

        std::string p_line3 = "! videoconvert ! video/x-raw, format=BGR ";  // Convert to standard format
        std::string p_line4 = "! appsink";

        std::string pipeline = p_line1 + p_line2 + p_line3 + p_line4;
        std::cout << "Pipeline: " << pipeline << std::endl;

        m_cap.open(pipeline, cv::CAP_GSTREAMER);
        if (!m_cap.isOpened()) {
            throw std::runtime_error("Failed to open camera via GStreamer.");
        }
    }

    bool isBlackImage(const cv::Mat& img)
    {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY); // Convert to grayscale
        return cv::countNonZero(gray) == 0;
    }

    /**
     * @brief Captures a frame from the video source.
     * @return std::optional<cv::Mat> containing the frame if successful, otherwise std::nullopt.
     */
    [[nodiscard]] inline std::optional<cv::Mat> getFrameImpl()
    {
        if (!m_cap.isOpened()) {
            throw std::runtime_error("Camera is not started.");
        }

        cv::Mat frame;
        while (frame.empty() || isBlackImage(frame))
        {
            if (!m_cap.read(frame))
            {
                return std::nullopt;
            }
        }

        return frame;
    }

    /**
     * @brief Stops the video capture and releases the camera.
     */
    inline void stopImpl()
    {
        if (m_cap.isOpened()) {
            m_cap.release();
        }
    }
};

#ifdef __linux__

/**
 * @brief Video source implementation using libcamera.
 */
class LibcameraVideoSource : public VideoSource<LibcameraVideoSource> {
private:
    float m_frame_rate;
    size_t m_frame_buffer_size;

    std::unique_ptr<libcamera::CameraManager> m_camera_manager;
    std::shared_ptr<libcamera::Camera> m_camera;
    std::unique_ptr<libcamera::CameraConfiguration> m_config;
    std::unique_ptr<libcamera::FrameBufferAllocator> m_frame_buffer_allocator;
    std::vector<std::unique_ptr<libcamera::Request>> m_requests;
    ConcurrentQueue<cv::Mat> m_frame_queue;
    std::atomic<bool> m_running{false};

public:
    /**
     * @brief Constructs a LibcameraVideoSource instance.
     */
    LibcameraVideoSource(std::string_view camera_id, size_t frame_buffer_size = 10);

    /**
     * @brief Destructs the video source and ensures cleanup.
     */
    ~LibcameraVideoSource();

    /**
     * @brief Starts the video source with a specified resolution.
     * @param frame_size Frame size.
     */
    void startImpl(const cv::Size frame_size);

    /**
     * @brief Captures a frame from the video source.
     * @return The captured frame if successful, otherwise std::nullopt.
     */
    [[nodiscard]] std::optional<cv::Mat> getFrameImpl();

    /**
     * @brief Stops the video source.
     */
    void stopImpl();

private:
    void requestComplete(libcamera::Request* request);
    void processFrame(libcamera::FrameBuffer* buffer);
};

#endif

/**
 * @brief Video source from a video file.
 */
class VideoFileSource : public VideoSource<VideoFileSource> {
private:
    std::filesystem::path m_file_path;
    cv::VideoCapture m_cap;
    bool m_repeat;
    cv::Size m_size;

public:
    /**
     * @brief Constructs a VideoFileSource.
     * @param file_path Path to the video file.
     * @param repeat Whether to loop the video after reaching the end.
     */
    inline VideoFileSource(const std::filesystem::path& file_path, bool repeat = false)
        : VideoSource(), m_file_path(std::filesystem::absolute(file_path)), m_cap(), m_repeat(repeat), m_size(0, 0) {}

    /**
     * @brief Starts the video capture.
     * @throws std::runtime_error if the video file fails to open.
     */
    inline void startImpl(const cv::Size frame_size)
    {
        if (m_cap.isOpened())
        {
            throw std::runtime_error("Video file already opened");
        }

        m_cap.open(m_file_path.string(), cv::CAP_FFMPEG);
        if (!m_cap.isOpened())
        {
            std::cout << cv::getBuildInformation() << std::endl;
            throw std::runtime_error("Failed to open video file: " + m_file_path.string());
        }

        m_size = frame_size;
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame if successful.
     * @retval std::nullopt if no frame captured.
     * @throws std::runtime_error on error.
     */
    [[nodiscard]] inline std::optional<cv::Mat> getFrameImpl()
    {
        if (!m_cap.isOpened())
        {
            throw std::runtime_error("Video file is not started");
        }

        cv::Mat frame;
        if (!m_cap.read(frame))
        {
            if (m_repeat)
            {
                m_cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Restart video
                if (!m_cap.read(frame))
                {
                    return std::nullopt; // Return empty if still failing
                }
            }
            else
            {
                return std::nullopt; // No frame available (end of video)
            }
        }

        frame = resizeWithPadding(frame, m_size);

        return frame;
    }

    /**
     * @brief Stops the video capture and releases the file.
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
