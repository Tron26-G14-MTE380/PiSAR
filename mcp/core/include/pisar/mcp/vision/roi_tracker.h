#pragma once
#include <optional>
#include <opencv2/opencv.hpp>

namespace pisar::mcp {

/**
 * @brief Tracks a moving ROI and predicts where it will be, with dynamic padding.
 */
class RoiTracker {
private:
    static constexpr double kMinVelocity = 0.0f;

    int m_min_padding;
    int m_max_padding;
    double m_max_velocity;
    std::optional<Eigen::Vector2d> m_last_center;
    std::optional<cv::Rect> m_last_roi;
    Eigen::Vector2d m_velocity;

public:
    RoiTracker(int min_padding = 30, int max_padding = 80, double max_velocity = 80.0f) :
        m_min_padding(min_padding),
        m_max_padding(max_padding),
        m_max_velocity(max_velocity),
        m_last_center(std::nullopt),
        m_last_roi(std::nullopt),
        m_velocity(Eigen::Vector2d::Zero())
        {}

    /**
     * @brief Submit an actual ROI.
     */
    void submit(const cv::Rect& actual_roi)
    {
        const Eigen::Vector2d center = getCenter(actual_roi);

        if (m_last_center.has_value())
        {
            m_velocity = center - m_last_center.value();
        } else {
            m_velocity = Eigen::Vector2d::Zero();
        }

        m_last_center = center;
        m_last_roi = actual_roi;
    }

    void reset()
    {
        m_last_center.reset();
        m_last_roi.reset();
        m_velocity = Eigen::Vector2d::Zero();
    }

    /**
     * @brief Get an estimated ROI to crop around.
     * @param image_size The full image dimensions (for clamping).
     * @return Optional predicted ROI or nullopt if not available.
     */
    [[nodiscard]] std::optional<cv::Rect> getEstimatedRoi(const cv::Size& image_size) const
    {
        if (!m_last_roi.has_value())
        {
            return std::nullopt;
        }

        return expandRectWithinBounds(*m_last_roi, image_size);
    }

private:

    [[nodiscard]] cv::Rect expandRectWithinBounds(const cv::Rect& rect, const cv::Size& img_size) const
    {
        double vx = std::clamp(m_velocity.x(), -m_max_velocity, m_max_velocity);
        double vy = std::clamp(m_velocity.y(), -m_max_velocity, m_max_velocity);

        double alpha_x = std::abs(vx) / m_max_velocity;
        double alpha_y = std::abs(vy) / m_max_velocity;

        const int dynamic_pad_x = static_cast<int>(m_min_padding + alpha_x * (m_max_padding - m_min_padding));
        const int dynamic_pad_y = static_cast<int>(m_min_padding + alpha_y * (m_max_padding - m_min_padding));

        const int pad_left  = (vx < 0) ? dynamic_pad_x : m_min_padding;
        const int pad_right = (vx > 0) ? dynamic_pad_x : m_min_padding;
        const int pad_top    = (vy < 0) ? dynamic_pad_y : m_min_padding;
        const int pad_bottom = (vy > 0) ? dynamic_pad_y : m_min_padding;

        const int x = std::max(0, rect.x - pad_left);
        const int y = std::max(0, rect.y - pad_top);
        const int right = std::min(img_size.width, rect.x + rect.width + pad_right);
        const int bottom = std::min(img_size.height, rect.y + rect.height + pad_bottom);

        return cv::Rect(x, y, right - x, bottom - y);
    }

    [[nodiscard]] static inline Eigen::Vector2d getCenter(const cv::Rect& r)
    {
        return Eigen::Vector2d(r.x + r.width * 0.5f, r.y + r.height * 0.5f);
    }
};

}