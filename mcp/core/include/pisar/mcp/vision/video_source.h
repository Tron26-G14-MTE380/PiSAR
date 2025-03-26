#pragma once

#include "pisar/mcp/vision/utils.h"
#include "pisar/mcp/utils/concurrent_queue.h"

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

struct CapturedFrame {
    using ClockT = std::chrono::steady_clock;
    using DurationT = std::chrono::duration<double>;
    using TimestampT = std::chrono::time_point<ClockT, DurationT>;

    cv::Mat frame;
    TimestampT timestamp;
};

/**
 * @brief CRTP base class for video sources.
 * @tparam TDerived The derived class implementing the interface.
 */
template <typename TDerived>
class VideoSource {
private:
    bool m_is_running;

public:
    /**
     * @brief Constructs a VideoSource with a specified resolution.
     */
    inline VideoSource() noexcept : m_is_running(false) {}

    ~VideoSource() {
        stop();
    }

    /**
     * @brief Starts the video source.
     * @param frame_size Frame size.
     */
    inline void start(const cv::Size frame_size)
    {
        static_cast<TDerived*>(this)->startImpl(frame_size);
        m_is_running = true;
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame is successful.
     * @retval std::nullopt if no frame captured.
     */
    [[nodiscard]] inline std::optional<CapturedFrame> getFrame()
    {
        return static_cast<TDerived*>(this)->getFrameImpl();
    }

    /**
     * @brief Stops the video source.
     */
    inline void stop()
    {
        static_cast<TDerived*>(this)->stopImpl();
        m_is_running = false;
    }

    /// @brief Get the camera capture rect indicating cropping from full-res.
    [[nodiscard]] inline cv::Rect getCaptureRect() const { return static_cast<TDerived*>(this)->getCaptureRectImpl(); }

    /// @brief Get whether the video source is running.
    [[nodiscard]] inline bool isRunning() const { return m_is_running; }
};

/**
 * @brief Video source that loads a static image and repeatedly returns it.
 */
class RepeatedImageFileSource : public VideoSource<RepeatedImageFileSource> {
private:
    std::string m_filePath;
    cv::Size m_original_image_size;
    cv::Mat m_img;

public:
    /**
     * @brief Constructs a RepeatedImageFileSource.
     * @param file_path Path to the image file.
     * @param width Image width.
     * @param height Image height.
     */
    inline RepeatedImageFileSource(const std::string& file_path)
        : VideoSource(), m_filePath(file_path), m_original_image_size(0, 0) {}

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

        m_original_image_size = m_img.size();

        cv::Mat resized;
        cv::resize(m_img, resized, frame_size);
        m_img = resized;
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame is successful.
     * @retval std::nullopt if no frame captured.
     */
    [[nodiscard]] inline std::optional<CapturedFrame> getFrameImpl()
    {
        if (m_img.empty()) {
            throw std::runtime_error("Image not loaded. Call start() first.");
        }
        return CapturedFrame { .frame = m_img.clone(), .timestamp = CapturedFrame::ClockT::now() };
    }

    /**
     * @brief Stops the image source.
     */
    inline void stopImpl() { m_img.release(); }

    /// @brief Get the camera capture rect indicating cropping from full-res.
    [[nodiscard]] inline cv::Rect getCaptureRect() const { return cv::Rect(0, 0, m_original_image_size.width, m_original_image_size.height); }
};

/**
 * @brief Video source from a camera feed.
 */
class CvCameraVideoSource : public VideoSource<CvCameraVideoSource> {
private:
    int m_camera_id;
    cv::Size m_original_size;
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

        m_original_size = {m_cap.get(cv::CAP_PROP_FRAME_WIDTH), m_cap.get(cv::CAP_PROP_FRAME_HEIGHT)};

        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame is successful.
     * @retval std::nullopt if no frame captured.
     * @throws std::runtime_error on error.
     */
    [[nodiscard]] inline std::optional<CapturedFrame> getFrameImpl()
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

        return CapturedFrame { .frame = frame, .timestamp = CapturedFrame::ClockT::now() };
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

    /// @brief Get the camera capture rect indicating cropping from full-res.
    [[nodiscard]] inline cv::Rect getCaptureRect() const { return cv::Rect(0, 0, m_original_size.width, m_original_size.height); }
};

#ifdef __linux__

/**
 * @brief Video source implementation using libcamera.
 */
class LibcameraVideoSource : public VideoSource<LibcameraVideoSource> {
private:
    float m_frame_rate;
    size_t m_frame_buffer_size;

    cv::Rect m_capture_rect;

    std::unique_ptr<libcamera::CameraManager> m_camera_manager;
    std::shared_ptr<libcamera::Camera> m_camera;
    std::unique_ptr<libcamera::CameraConfiguration> m_config;
    std::unique_ptr<libcamera::FrameBufferAllocator> m_frame_buffer_allocator;
    std::vector<std::unique_ptr<libcamera::Request>> m_requests;
    ConcurrentQueue<CapturedFrame> m_frame_queue;
    std::atomic<bool> m_running{false};

public:
    /**
     * @brief Constructs a LibcameraVideoSource instance.
     */
    LibcameraVideoSource(std::string_view camera_id, float frame_rate = 90, size_t frame_buffer_size = 10);

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
    [[nodiscard]] std::optional<CapturedFrame> getFrameImpl();

    /**
     * @brief Stops the video source.
     */
    void stopImpl();

    /// @brief Get the camera capture rect indicating cropping from full-res.
    [[nodiscard]] inline cv::Rect getCaptureRect() const { return m_capture_rect; }

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
    cv::Size m_original_size;
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
        : VideoSource(),
          m_file_path(std::filesystem::absolute(file_path)),
          m_cap(),
          m_original_size(0, 0),
          m_repeat(repeat),
          m_size(0, 0)
          {}

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

        m_original_size = {m_cap.get(cv::CAP_PROP_FRAME_WIDTH), m_cap.get(cv::CAP_PROP_FRAME_HEIGHT)};
        m_size = frame_size;
    }

    /**
     * @brief Captures a frame from the video source.
     * @retval cv::Mat The captured frame if successful.
     * @retval std::nullopt if no frame captured.
     * @throws std::runtime_error on error.
     */
    [[nodiscard]] inline std::optional<CapturedFrame> getFrameImpl()
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

        return CapturedFrame { .frame = frame, .timestamp = CapturedFrame::ClockT::now() };
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

    /// @brief Get the camera capture rect indicating cropping from full-res.
    [[nodiscard]] inline cv::Rect getCaptureRect() const { return cv::Rect(0, 0, m_original_size.width, m_original_size.height); }
};

}
