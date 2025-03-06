#pragma once

#include <cmath>
#include <algorithm>

namespace pisar {

template<typename T>
class LowPassFilter {
private:
    T m_output;
    float m_e_pow;

public:
    /**
     * @brief Construct the low pass filter.
     *
     * @param cutoff_freq The LPF cutoff frequency.
     * @param delta_time_s The sample time between inputs.
     */
    inline constexpr LowPassFilter(float cutoff_freq, float delta_time_s) :
        m_output(0),
	    m_e_pow(std::clamp(1 - std::exp(-delta_time_s * 2 * std::numbers::pi_v<float> * cutoff_freq), 0.0f, 1.0f))
    {}

    /**
     * @brief Update the LPF filter.
     *
     * @param input The next input
     * @return The filtered value.
     */
    inline constexpr T update(T input)
    {
        m_output += (input - m_output) * m_e_pow;
        return m_output;
    }

    /// @brief Get the filter output
    [[nodiscard]] inline constexpr T getOutput() const
    {
        return m_output;
    }
};

}