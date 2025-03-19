#include "pisar/vision/video_source.h"

#ifdef __linux__
#include <libcamera/stream.h>
#include <libcamera/control_ids.h>

#include <sys/mman.h>
#endif

#include <sstream>

namespace pisar::mcp {

#ifdef __linux__

/**
 * @brief Constructs a LibcameraVideoSource instance.
 */
LibcameraVideoSource::LibcameraVideoSource(std::string_view camera_id, float frame_rate, size_t frame_buffer_size)
    : m_camera_manager(std::make_unique<libcamera::CameraManager>()), m_frame_rate(frame_rate), m_frame_buffer_size(frame_buffer_size)
{
    if (m_camera_manager->start())
    {
        throw std::runtime_error("Failed to start libcamera::CameraManager");
    }

    auto cameras = m_camera_manager->cameras();
    if (cameras.empty())
    {
        throw std::runtime_error("No cameras found");
    }

    for (auto& camera : m_camera_manager->cameras())
    {
        if (camera->id() == camera_id)
        {
            m_camera = camera;
            break;
        }
    }

    if (!m_camera)
    {
        std::stringstream err;
        err << "Camera with id " << camera_id << " not found" << std::endl;
        throw std::runtime_error(err.str());
    }

    if (m_camera->acquire())
    {
        throw std::runtime_error("Failed to acquire camera");
    }
}

/**
 * @brief Destructs the video source and ensures cleanup.
 */
LibcameraVideoSource::~LibcameraVideoSource()
{
    stopImpl();
    if (m_camera)
    {
        m_camera->release();
    }
    m_camera_manager->stop();
}

/**
 * @brief Starts the video source with a specified resolution.
 * @param frame_size Frame size.
 */
void LibcameraVideoSource::startImpl(const cv::Size frame_size)
{
    if (m_running.exchange(true))
    {
        return; // Already running
    }

    m_config = m_camera->generateConfiguration({ libcamera::StreamRole::VideoRecording });
    if (!m_config)
    {
        throw std::runtime_error("Failed to generate camera configuration");
    }

    auto& stream_config = m_config->at(0);
    stream_config.size.width = frame_size.width;
    stream_config.size.height = frame_size.height;
    stream_config.pixelFormat = libcamera::formats::YUV420;

    if (m_config->validate() == libcamera::CameraConfiguration::Invalid)
    {
        throw std::runtime_error("Invalid camera configuration");
    }

    if (m_camera->configure(m_config.get()) < 0)
    {
        throw std::runtime_error("Failed to configure camera");
    }

    libcamera::Stream* stream = stream_config.stream();
    m_frame_buffer_allocator = std::make_unique<libcamera::FrameBufferAllocator>(m_camera);

    if (m_frame_buffer_allocator->allocate(stream) < 0)
    {
        throw std::runtime_error("Failed to allocate buffers");
    }

    for (const std::unique_ptr<libcamera::FrameBuffer>& buffer : m_frame_buffer_allocator->buffers(stream))
    {
        std::unique_ptr<libcamera::Request> request = m_camera->createRequest();
        if (!request)
        {
            throw std::runtime_error("Failed to create camera request");
        }

        if (request->addBuffer(stream, buffer.get()) < 0)
        {
            throw std::runtime_error("Can't set buffer for request");
        }

        m_requests.push_back(std::move(request));
    }

    // Connect the requestCompleted signal to handle finished frames
    m_camera->requestCompleted.connect(this, &LibcameraVideoSource::requestComplete);

    const int64_t frame_period_us = 1'000'000 / m_frame_rate;

    libcamera::ControlList cam_controls;
    cam_controls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const std::int64_t, 2>({frame_period_us, frame_period_us}));

    // Start the camera
    if (m_camera->start(&cam_controls) < 0)
    {
        throw std::runtime_error("Failed to start camera");
    }

    // Queue the requests
    for (auto& request: m_requests)
    {
        if (m_camera->queueRequest(request.get()) < 0)
        {
            throw std::runtime_error("Failed to queue request");
        }
    }
}

/**
 * @brief Handles completed requests asynchronously.
 * @param request The completed request.
 */
void LibcameraVideoSource::requestComplete(libcamera::Request* request)
{
    if (request->status() != libcamera::Request::RequestComplete)
    {
        std::cerr << "Request not completed successfully" << std::endl;
        return;
    }

    for (auto& [stream, buffer] : request->buffers())
    {
        processFrame(buffer);
    }

    // Recycle the request for continuous streaming
    request->reuse(libcamera::Request::ReuseBuffers);
    if (m_camera->queueRequest(request) < 0)
    {
        throw std::runtime_error("Failed to re-queue request");
    }
}

/**
 * @brief Processes a captured frame and converts it to OpenCV format.
 * @param buffer The frame buffer containing image data.
 */
void LibcameraVideoSource::processFrame(libcamera::FrameBuffer* buffer)
{
    const libcamera::FrameMetadata& metadata = buffer->metadata();
    if (metadata.status != libcamera::FrameMetadata::FrameSuccess)
    {
        std::cerr << "Frame capture failed" << std::endl;
        return;
    }

    // Create OpenCV Mats for each plane
    std::array<cv::Mat, 3> planes;

    if (buffer->planes().size() != planes.size())
    {
        throw std::runtime_error("Invalid number of planes");
    }

    auto& stream_config = m_config->at(0);
    int width = stream_config.size.width;
    int height = stream_config.size.height;

    // Verify that all planes are sequential on the same fd
    int total_length = 0;
    const int base_fd = buffer->planes()[0].fd.get();  // Get FD of the first plane
    off_t expected_offset = buffer->planes()[0].offset;

    for (size_t i = 0; i < buffer->planes().size(); ++i)
    {
        const auto& plane = buffer->planes()[i];

        // Ensure all planes share the same FD
        if (plane.fd.get() != base_fd)
        {
            std::stringstream err;
            err << "Plane " << i << " uses a different FD! Expected: " << base_fd
                << ", Found: " << plane.fd.get();
            throw std::runtime_error(err.str());

        }

        // Ensure planes are sequential
        if (plane.offset != expected_offset)
        {
            std::stringstream err;
            err << "Plane " << i << " is not sequential! Expected offset: "
                << expected_offset << ", Found: " << plane.offset;
            throw std::runtime_error(err.str());
        }

        total_length += plane.length;
        expected_offset += plane.length;  // Move expected offset forward
    }

    void* mapped_memory = mmap(nullptr, total_length, PROT_READ, MAP_SHARED, base_fd, buffer->planes()[0].offset);
    if (mapped_memory == MAP_FAILED)
    {
        std::cerr << "mmap failed: " << strerror(errno) << std::endl;
        return;
    }

    // Create opencv Mat objects from mapped memory
    for (size_t i = 0; i < buffer->planes().size(); ++i)
    {
        const auto& plane = buffer->planes()[i];
        uint8_t* plane_memory = reinterpret_cast<uint8_t*>(mapped_memory) + plane.offset;
        const int plane_width = i == 0 ? width : width / 2;
        const int plane_height = i == 0 ? height : height / 2;
        const int stride = plane.length / plane_height;

        planes[i] = cv::Mat(plane_height, plane_width, CV_8UC1, plane_memory, stride).clone();
    }

    munmap(mapped_memory, total_length);

    // Resize u and v to y plane size so they can be merged
    cv::resize(planes[1], planes[1], { width, height }, 0, 0);
	cv::resize(planes[2], planes[2], { width, height }, 0, 0);

    // Merge into a single YUV image
    cv::Mat yuv_image;
    cv::merge(planes.data(), planes.size(), yuv_image);

    // Convert to BGR
    cv::Mat bgr_image;
    cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR);

    while (m_frame_queue.size() >= m_frame_buffer_size)
    {
        m_frame_queue.pop();
    }

    // Push to frame queue
    m_frame_queue.push(std::move(bgr_image));
}

/**
 * @brief Retrieves the most recent frame.
 * @return The latest frame as cv::Mat if available, otherwise std::nullopt.
 */
[[nodiscard]] std::optional<cv::Mat> LibcameraVideoSource::getFrameImpl()
{
    return m_frame_queue.pop();
}

/**
 * @brief Stops the video source.
 */
void LibcameraVideoSource::stopImpl()
{
    if (!m_running.exchange(false))
    {
        return; // Already stopped
    }

    m_camera->requestCompleted.disconnect(this, &LibcameraVideoSource::requestComplete);
    m_camera->stop();
    m_frame_buffer_allocator->free(m_config->at(0).stream());
    m_frame_buffer_allocator.reset();
    m_config.reset();
    m_requests.clear();
    m_frame_queue.clear();
}

#endif

}
