/**
 * @file thinning_ma_ren.cpp
 * @author Ashkan Ebrahimi (a28ebrah@uwaterloo.ca)
 * @brief An implementation of a skeletonization/thinning algorithm proposed in the paper 
 * "A novel fully parallel skeletonization algorithm". Full credits to Jun Ma, Xunhuan Ren, Viktar Yurevich Tsviatkou 
 * and Valery Kanstantinavich Kanapelka for the theory behind the algorithm.
 */

#include "pisar/vision/thinning.h"

#include <opencv2/opencv.hpp>
#include <easy/profiler.h>

#include <bitset>
#include <tuple>
#include <thread>
#include <atomic>
#include <barrier>
#include <vector>

namespace pisar::mcp {

namespace detail {
    template<std::size_t...tkIs, class TTuple>
    static constexpr inline auto toBitset(std::index_sequence<tkIs...>, TTuple&& tuple)
    {
        constexpr auto size = sizeof...(tkIs);
        using expand = int[];
        unsigned long bits = 0;
        void(expand {
            0,
            ((bits |= std::get<tkIs>(tuple) ? 1ul << (size - tkIs - 1) : 0),0)...
        });
        return std::bitset<size>(bits);
    }
}

template<class...TBools>
static constexpr inline auto toBitset(TBools&&...bools)
{
    return detail::toBitset(std::make_index_sequence<sizeof...(TBools)>(),
                             std::make_tuple(bool(bools)...));
}


/**
 * @brief 
 * @param img The binary image (CV_8UC1, values should be 0 or 1).
 * @param p The pixel coordinate (x, y).
 * @return A bitset representing the neighborhood.
 */

inline static std::bitset<8> getPackedImmediateNeighbors(const cv::Mat& img, cv::Point p) {
    const uint8_t* row_above = img.ptr<uint8_t>(p.y - 1);
    const uint8_t* row_center = img.ptr<uint8_t>(p.y);
    const uint8_t* row_below = img.ptr<uint8_t>(p.y + 1);

    return std::bitset<8>(
        (row_above[p.x])            |   // P1
        (row_above[p.x + 1] << 1)   |   // P2
        (row_center[p.x + 1] << 2)  |   // P3
        (row_below[p.x + 1] << 3)   |   // P4
        (row_below[p.x]     << 4)   |   // P5
        (row_below[p.x - 1] << 5)   |   // P6
        (row_center[p.x - 1] << 6)  |   // P7
        (row_above[p.x - 1] << 7)       // P8
    );
}

static constexpr uint8_t kNeighborBitMapping[5][5] = {
    {0, 0, 1, 2, 0},
    {3, 4, 5, 6, 7},
    {8, 9, 0, 10, 11},
    {12, 13, 14, 15, 16},
    {0, 17, 18, 19, 0}
};

/**
 * @brief Extracts a 20-pixel neighborhood and encodes it as a bitset.
 * @param img The binary image (CV_8UC1, values should be 0 or 1).
 * @param p The center pixel coordinate.
 * @return A 20-bit bitset representing the neighborhood.
 */
inline static std::bitset<20> getPackedNeighbors(const cv::Mat& img, cv::Point p) {
    const uint8_t* row0 = img.ptr<uint8_t>(p.y - 2);
    const uint8_t* row1 = img.ptr<uint8_t>(p.y - 1);
    const uint8_t* row2 = img.ptr<uint8_t>(p.y);
    const uint8_t* row3 = img.ptr<uint8_t>(p.y + 1);
    const uint8_t* row4 = img.ptr<uint8_t>(p.y + 2);

    return toBitset(
                       row0[p.x - 1], row0[p.x], row0[p.x + 1],
        row1[p.x - 2], row1[p.x - 1], row1[p.x], row1[p.x + 1], row1[p.x + 2],
        row2[p.x - 2], row2[p.x - 1],            row2[p.x + 1], row2[p.x + 2],
        row3[p.x - 2], row3[p.x - 1], row3[p.x], row3[p.x + 1], row3[p.x + 2],
                       row4[p.x - 1], row4[p.x], row4[p.x + 1]
    );
}

inline uint8_t num_neighbours_set(const std::bitset<8> packed_neighbors)
{
    return packed_neighbors.count();
}

inline uint8_t num_ordered_pairs(const std::bitset<8> packed_neighbors)
{
    const std::bitset<8> shifted = (packed_neighbors << 1) | (packed_neighbors >> 7); // Circular shift left
    return ((~packed_neighbors) & shifted).count(); // Count 0->1 transitions
}


static inline bool checkTemplateA(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 1,    1, 1,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0,    1, 0,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    return (neighbors & mask) == pattern;
}

static inline bool checkTemplateB(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0,    1, 0,
        0, 0, 1, 0, 0,
           0, 1, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0,    1, 0,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    return (neighbors & mask) == pattern;
}

static inline bool checkTemplateC(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 0, 1, 0, 0,
        0, 1,    1, 0,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0,    1, 0,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    constexpr std::bitset<20> t_mask = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 1,
        0, 0,    0, 1,
        0, 0, 0, 0, 1,
           1, 1, 1
    );

    return (neighbors & t_mask).count() >= 2;
}

static inline bool checkTemplateD(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           1, 1, 1,
        0, 1, 1, 1, 0,
        0, 1,    1, 0,
        0, 0, 1, 1, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0,    1, 0,
        0, 0, 1, 1, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 + 2][2]] != pattern[kNeighborBitMapping[2 + 2][2 + 1]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 + 2]] != pattern[kNeighborBitMapping[2 + 1][2 + 2]])
    {
        return false;
    }

    constexpr std::bitset<20> s_mask = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0,    0, 1,
        0, 0, 0, 0, 1,
           0, 1, 1
    );

    return (neighbors & s_mask).count() > 1;
}

static inline bool checkTemplateE(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        1, 1, 1, 0, 0,
        1, 1,    1, 0,
        1, 1, 1, 1, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 1,    1, 0,
        0, 0, 1, 1, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 + 2][2]] != pattern[kNeighborBitMapping[2 + 2][2 + 1]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 + 2]] != pattern[kNeighborBitMapping[2 + 1][2 + 2]])
    {
        return false;
    }

    constexpr std::bitset<20> s_mask = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0,    0, 1,
        0, 0, 0, 0, 1,
           0, 1, 1
    );

    return (neighbors & s_mask).count() > 1;
}

static inline bool checkTemplateF(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        1, 1, 1, 1, 0,
        1, 1,    1, 0,
        1, 1, 1, 0, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 1, 1, 0,
        0, 1,    1, 0,
        0, 0, 0, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 - 2][2]] != pattern[kNeighborBitMapping[2 - 2][2 + 1]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 + 2]] != pattern[kNeighborBitMapping[2 - 1][2 + 2]])
    {
        return false;
    }

    return true;
}

static inline bool checkTemplateG(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 1, 1, 1, 1,
        0, 1,    1, 1,
        0, 0, 1, 1, 1,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 1, 1, 0, 0,
        0, 1,    1, 0,
        0, 0, 0, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 - 2][2]] != pattern[kNeighborBitMapping[2 - 2][2 - 1]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 - 2]] != pattern[kNeighborBitMapping[2 - 1][2 - 2]])
    {
        return false;
    }

    return true;
}

static inline bool checkTemplateH(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 0, 1, 1, 1,
        0, 1,    1, 1,
        0, 1, 1, 1, 1,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 0, 0, 0,
        0, 1,    1, 0,
        0, 1, 1, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 - 2]] != pattern[kNeighborBitMapping[2 + 1][2 - 2]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 + 2][2]] != pattern[kNeighborBitMapping[2 - 2][2 - 1]])
    {
        return false;
    }

    return true;
}

static inline bool checkTemplateI(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           1, 1, 1,
        0, 1, 1, 1, 0,
        0, 1,    1, 0,
        0, 1, 1, 0, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 1, 0, 0,
        0, 1,    0, 0,
        0, 1, 1, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 - 2]] != pattern[kNeighborBitMapping[2 + 1][2 - 2]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 + 2][2]] != pattern[kNeighborBitMapping[2 - 2][2 - 1]])
    {
        return false;
    }

    return true;
}

static inline bool checkTemplateJ(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 0, 1, 1, 0,
        0, 1,    1, 0,
        0, 1, 1, 1, 0,
           1, 1, 1
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 1, 1, 0,
        0, 0,    1, 0,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 - 2][2]] != pattern[kNeighborBitMapping[2 - 2][2 + 1]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 + 2]] != pattern[kNeighborBitMapping[2 - 1][2 + 2]])
    {
        return false;
    }

    return true;
}

static inline bool checkTemplateK(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 1, 1, 0, 0,
        0, 1,    1, 0,
        0, 1, 1, 1, 0,
           1, 1, 1
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 1, 1, 0, 0,
        0, 1,    0, 0,
        0, 0, 1, 0, 0,
           0, 0, 0
    );

    if ((neighbors & mask) != pattern)
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2 - 2][2]] != pattern[kNeighborBitMapping[2 - 2][2 - 1]])
    {
        return false;
    }

    if (pattern[kNeighborBitMapping[2][2 - 2]] != pattern[kNeighborBitMapping[2 - 1][2 - 2]])
    {
        return false;
    }

    return true;
}

static inline bool checkTemplateL(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 1, 1, 1, 0,
        0, 1,    1, 0,
        0, 1, 1, 1, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 1, 1, 0, 0,
        0, 0,    1, 0,
        0, 0, 0, 1, 0,
           0, 0, 0
    );

    return (neighbors & mask) == pattern;
}

static inline bool checkTemplateM(const std::bitset<20> neighbors)
{
    constexpr std::bitset<20> mask = toBitset(
           0, 0, 0,
        0, 1, 1, 1, 0,
        0, 1,    1, 0,
        0, 1, 1, 1, 0,
           0, 0, 0
    );

    constexpr std::bitset<20> pattern = toBitset(
           0, 0, 0,
        0, 0, 1, 1, 0,
        0, 1,    0, 0,
        0, 1, 0, 0, 0,
           0, 0, 0
    );

    return (neighbors & mask) == pattern;
}

static bool shouldDeletePixel(const cv::Mat& img, int r, int c)
{
    const bool foreground = img.at<uint8_t>(r, c);
    if (!foreground)
    {
        return false;
    }
    
    const std::bitset<8> immediate_neighbors = getPackedImmediateNeighbors(img, cv::Point{c, r});
    const uint8_t A = num_ordered_pairs(immediate_neighbors);
    const uint8_t B = num_neighbours_set(immediate_neighbors);

    if (A == 1 && B >= 2 && B <= 6) // Global zhang-suan condition
    {
        const std::bitset<20> neighbors = getPackedNeighbors(img, cv::Point{c, r});
        const bool restoring = checkTemplateA(neighbors) || checkTemplateB(neighbors);
        if (restoring == false || checkTemplateD(neighbors))
        {
            return true;
        }
    }
    else if (A == 2 && (B == 4 || B == 5))
    {
        const std::bitset<20> neighbors = getPackedNeighbors(img, cv::Point{c, r});
        const std::array<bool (*)(const std::bitset<20>), 10> extra_deletion_template_checkers = {
            checkTemplateD, checkTemplateE, checkTemplateF, checkTemplateG, checkTemplateH, 
            checkTemplateI, checkTemplateJ, checkTemplateK, checkTemplateL, checkTemplateM
        };

        for (auto deletion_template_checker : extra_deletion_template_checkers)
        {
            if (deletion_template_checker(neighbors))
            {
                return true;
            }
        }
    }

    return false;
}

static inline bool shouldDeletePixelPostProcessing(const cv::Mat& img, int r, int c) 
{
    const std::bitset<8> neighbors = getPackedImmediateNeighbors(img, cv::Point{c, r});

    constexpr std::bitset<8> mask1 = (1 << 5) | (1 << 2) | (1 << 0); 
    constexpr std::bitset<8> mask2 = (1 << 7) | (1 << 4) | (1 << 2); 
    constexpr std::bitset<8> mask3 = (1 << 1) | (1 << 6) | (1 << 4); 
    constexpr std::bitset<8> mask4 = (1 << 3) | (1 << 0) | (1 << 6);

    constexpr std::bitset<8> pattern1 = (1 << 2) | (1 << 0); 
    constexpr std::bitset<8> pattern2 = (1 << 4) | (1 << 2); 
    constexpr std::bitset<8> pattern3 = (1 << 6) | (1 << 4); 
    constexpr std::bitset<8> pattern4 = (1 << 0) | (1 << 6);

    return (
        ((neighbors & mask1) == pattern1) || 
        ((neighbors & mask2) == pattern2) || 
        ((neighbors & mask3) == pattern3) ||  
        ((neighbors & mask4) == pattern4)
    );
}

static void postProcessing(cv::Mat& img)
{
    for (int r = 2; r < img.rows - 2; ++r)
    {
        for (int c = 2; c < img.cols - 2; ++c)
        {
            if (shouldDeletePixelPostProcessing(img, r, c))
            {
                img.at<uint8_t>(r, c) = 0;
            }
        }
    }
}

static void thinningMaRenIterativeImpl(cv::Mat& img)
{
    cv::Mat delete_matrix = cv::Mat::zeros(img.size(), CV_8UC1);

    while (true)
    {
        bool flag = false;
        for (int r = 2; r < img.rows - 2; ++r)
        {
            for (int c = 2; c < img.cols - 2; ++c)
            {
                if (shouldDeletePixel(img, r, c))
                {
                    delete_matrix.at<uint8_t>(r, c) = 1;
                    flag = true;
                }
            }
        }
        
        if (flag == false)
        {
            break;
        }

        img -= delete_matrix;
        delete_matrix.setTo(0);
    }
    
    postProcessing(img);
}


#include <iostream>

/// @brief Class for parallel image thinning
class MaRenParallelThinning {
private:
    cv::Mat m_image;                                    ///< Original input image (read-only)
    cv::Mat m_new_image;                                ///< Updated image after each iteration
    int m_num_threads;                                  ///< Number of worker threads
    std::atomic<bool> m_deleted;                        ///< Flag to check if any pixel was deleted
    std::atomic<bool> m_stop;                    ///< Flag to check if any pixel was deleted
    std::barrier<std::function<void()>> m_barrier;      ///< Synchronization barrier for iterations

public:
    /**
     * @brief Constructs the thinning processor
     * 
     * @param num_threads Number of worker threads
     */
    MaRenParallelThinning(int num_threads)
        : m_image(), m_new_image(), m_num_threads(num_threads), m_deleted(false), m_stop(false),
          m_barrier(m_num_threads, [this] { this->assembleImage(); })
    {}

    /// @brief Runs the thinning algorithm
    void run(const cv::Mat& img, cv::Mat& output) 
    {
        m_image = img.clone();
        m_new_image = img.clone();

        m_deleted.store(false, std::memory_order_relaxed);
        m_stop.store(false, std::memory_order_relaxed);

        std::vector<std::thread> workers;
        workers.reserve(m_num_threads);

        const int totalRows = m_image.rows - 4;  // Exclude first 2 and last 2 rows
        const int rowsPerThread = totalRows / m_num_threads;
        const int remainder = totalRows % m_num_threads;

        int start_row = 2; // Begin at row index 2
        for (int i = 0; i < m_num_threads; ++i) 
        {
            int end_row = start_row + rowsPerThread + (i < remainder ? 1 : 0); // Distribute remainder
            workers.emplace_back(&MaRenParallelThinning::workerThread, this, start_row, end_row);
            start_row = end_row; // Move to next chunk
        }

        for (auto& t : workers) 
        {
            t.join();
        }

        output = m_new_image;
        postProcessing(output);
    }

private:
    /**
     * @brief Worker thread function
     * 
     * @param start_row Start row of the assigned segment
     * @param end_row End row of the assigned segment
     */
    void workerThread(int start_row, int end_row) {
        cv::Mat local_delete_matrix = cv::Mat::zeros(cv::Size(m_image.cols, end_row - start_row), CV_8UC1);
        while (true) {
            
            bool local_deleted = false;

            for (int r = start_row; r < end_row; ++r) 
            {
                for (int c = 2; c < m_image.cols - 2; ++c) 
                {
                    if (shouldDeletePixel(m_image, r, c)) 
                    {
                        local_delete_matrix.at<uint8_t>(r - start_row, c) = 1;
                        local_deleted = true;
                    }
                }
            }

            if (local_deleted) 
            {
                m_deleted.store(true, std::memory_order_relaxed);
            }
            
            // Extract the region from m_new_image that corresponds to this thread's work area
            cv::Mat image_region = m_new_image.rowRange(start_row, end_row);
            image_region -= local_delete_matrix; // Apply deletions in one step
            local_delete_matrix.setTo(0);

            m_barrier.arrive_and_wait();

            if (m_stop.load(std::memory_order_relaxed)) 
            {
                break;  // Stop when no pixels are deleted
            }
        }
    }

    /// @brief Assembles the new image for the next iteration
    void assembleImage() 
    {
        std::swap(m_image, m_new_image);
        m_new_image = m_image.clone();

        m_stop.store(!m_deleted.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_deleted.store(false, std::memory_order_relaxed);
    }
};

void thinningMaRenIterative(cv::InputArray img, cv::OutputArray output)
{
    EASY_FUNCTION();

    cv::Mat processed = img.getMat().clone();
    CV_CheckTypeEQ(processed.type(), CV_8UC1, "");
    // Enforce the range of the input image to be in between 0 - 255
    processed /= 255;

    thinningMaRenIterativeImpl(processed);

    processed *= 255;
    output.assign(processed);
}

void thinningMaRenParallel(cv::InputArray img, cv::OutputArray output)
{
    EASY_FUNCTION();

    cv::Mat processed = img.getMat().clone();
    CV_CheckTypeEQ(processed.type(), CV_8UC1, "");
    // Enforce the range of the input image to be in between 0 - 255
    processed /= 255;

    // TODO: Find a better way to set threads based on host processors but lets hardcode 4 for now (rpi5 has 4 cores)

    cv::Mat output_img;
    auto thinning = MaRenParallelThinning(4);
    thinning.run(processed, output_img);

    output_img *= 255;
    output.assign(output_img);
}

}