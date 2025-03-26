#pragma once

#include "pisar/mcp/vision/utils.h"
#include "pisar/mcp/vision/camera.h"
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
class CameraSource {
private:
    bool m_is_running;

protected:
    CameraCaptureConfig m_capture_config;

public:
    /**
     * @brief Constructs a CameraSource with a specified resolution.
     * @param capture_config The camera capture configuration.
     */
    inline CameraSource(const CameraCaptureConfig& capture_config) noexcept :
        m_is_running(false), m_capture_config(capture_config) {}

    ~CameraSource()
    {
        stop();
    }

    /**
     * @brief Starts the video source.
     */
    inline void start()
    {
        static_cast<TDerived*>(this)->startImpl();
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

    /// @brief Get whether the video source is running.
    [[nodiscard]] inline bool isRunning() const { return m_is_running; }

    /// @brief Returns the capture configuration.
    [[nodiscard]] inline const CameraCaptureConfig& captureConfig() const { return m_capture_config; }
};

static inline cv::Mat transformImageToCameraConfig(const cv::Mat& input, const CameraCaptureConfig& capture_config)
{
    // Crop input to match aspect ratio of binned resolution
    // Resize to binned resolution.

    const double binned_aspect = static_cast<double>(capture_config.binnedSize().width) / capture_config.binnedSize().height;
    const double input_aspect = static_cast<double>(input.cols) / input.rows;

    cv::Mat binned_image = input;
    if (std::abs(binned_aspect - input_aspect) > 1e-3)
    {
        binned_image = resizeCropMaintainRatio(input, capture_config.binnedSize());
    }

    // Crop the configured capture region (e.g., center or offset)
    cv::Mat captured_image = binned_image(capture_config.captureCrop());

    // Resize to downscaled resolution
    cv::Mat output_image;
    cv::resize(captured_image, output_image, capture_config.downscaledSize(), 0, 0, cv::INTER_LINEAR);

    return output_image;
}

/**
 * @brief Video source that loads a static image and repeatedly returns it.
 */
class RepeatedImageFileCameraSource : public CameraSource<RepeatedImageFileCameraSource> {
private:
    std::string m_filePath;
    cv::Mat m_img;

public:
    /**
     * @brief Constructs a RepeatedImageFileCameraSource.
     * @param capture_config The camera capture configuration.
     * @param file_path Path to the image file.
     * @param width Image width.
     * @param height Image height.
     */
    inline RepeatedImageFileCameraSource(const CameraCaptureConfig& capture_config, const std::string& file_path)
        : CameraSource(capture_config), m_filePath(file_path) {}

    /**
     * @brief Starts the image source by loading the file.
     * @throws std::runtime_error if the image fails to load.
     */
    inline void startImpl()
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

        m_img = transformImageToCameraConfig(m_img, m_capture_config);
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
};

/**
 * @brief Video source from a video file.
 */
class VideoFileCameraSource : public CameraSource<VideoFileCameraSource> {
private:
    std::filesystem::path m_file_path;
    cv::VideoCapture m_cap;
    bool m_repeat;

public:
    /**
     * @brief Constructs a VideoFileCameraSource.
     * @param capture_config The camera capture configuration.
     * @param file_path Path to the video file.
     * @param repeat Whether to loop the video after reaching the end.
     */
    inline VideoFileCameraSource(const CameraCaptureConfig& capture_config, const std::filesystem::path& file_path, bool repeat = false)
        : CameraSource(capture_config),
            m_file_path(std::filesystem::absolute(file_path)),
            m_cap(),
            m_repeat(repeat)
            {}

    /**
     * @brief Starts the video capture.
     * @throws std::runtime_error if the video file fails to open.
     */
    inline void startImpl()
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

        frame = transformImageToCameraConfig(frame, m_capture_config);

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
};

/**
 * @brief Video source from a camera feed.
 */
class CvCameraSource : public CameraSource<CvCameraSource> {
private:
    int m_camera_id;
    cv::VideoCapture m_cap;

public:
    /**
     * @brief Constructs a CvCameraSource.
     * @param capture_config The camera capture configuration.
     * @param camera_id Camera id to select.
     */
    inline CvCameraSource(const CameraCaptureConfig& capture_config, int camera_id) :
        CameraSource(capture_config), m_camera_id(camera_id), m_cap() {}

    /**
     * @brief Starts the video capture.
     * @throws std::runtime_error if the camera fails to open.
     */
    inline void startImpl()
    {
        if (m_cap.isOpened())
        {
            throw std::runtime_error("Camera already opened");
        }

        if (!m_cap.open(m_camera_id))
        {
            throw std::runtime_error("Failed to open camera");
        }

        m_cap.set(cv::CAP_PROP_FRAME_WIDTH,  m_capture_config.binnedSize().width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_capture_config.binnedSize().height);
        m_cap.set(cv::CAP_PROP_FPS, m_capture_config.framerate());
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

        cv::Mat resized_output;
        cv::resize(frame, resized_output, m_capture_config.downscaledSize(), 0, 0, cv::INTER_LINEAR);

        return CapturedFrame { .frame = resized_output, .timestamp = CapturedFrame::ClockT::now() };
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

#ifdef __linux__

/**
 * @brief Video source implementation using libcamera.
 */
class LibcameraCameraSource : public CameraSource<LibcameraCameraSource> {
private:
    float m_frame_rate;
    size_t m_frame_buffer_size;

    std::unique_ptr<libcamera::CameraManager> m_camera_manager;
    std::shared_ptr<libcamera::Camera> m_camera;
    std::unique_ptr<libcamera::CameraConfiguration> m_config;
    std::unique_ptr<libcamera::FrameBufferAllocator> m_frame_buffer_allocator;
    std::vector<std::unique_ptr<libcamera::Request>> m_requests;
    ConcurrentQueue<CapturedFrame> m_frame_queue;
    std::atomic<bool> m_running{false};

public:
    /**
     * @brief Constructs a LibcameraCameraSource instance.
     * @param capture_config The camera capture configuration.
     * @param camera_id The camera id to select.
     * @param frame_buffer_size The max number of frames in queue.
     */
    LibcameraCameraSource(const CameraCaptureConfig& capture_config, std::string_view camera_id, size_t frame_buffer_size = 10);

    /**
     * @brief Destructs the video source and ensures cleanup.
     */
    ~LibcameraCameraSource();

    /**
     * @brief Starts the video source with a specified resolution.
     * @param frame_size Frame size.
     */
    void startImpl();

    /**
     * @brief Captures a frame from the video source.
     * @return The captured frame if successful, otherwise std::nullopt.
     */
    [[nodiscard]] std::optional<CapturedFrame> getFrameImpl();

    /**
     * @brief Stops the video source.
     */
    void stopImpl();

private:
    void requestComplete(libcamera::Request* request);
    void processFrame(libcamera::FrameBuffer* buffer);
};

#endif

}
