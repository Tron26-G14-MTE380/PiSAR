#pragma once

#include <cmath>

namespace pisar {

template<typename T>
class LowPassFilter {
private:
    T m_output;
    float m_e_pow;

public:
    inline constexpr LowPassFilter(): m_output(0), m_e_pow(0) {}

    inline constexpr LowPassFilter(float cutoff_freq, float delta_time) :
        m_output(0),
	    m_e_pow(1-exp(-delta_time * 2 * M_PI * cutoff_freq))
    {}

    inline constexpr T update(T input)
    {
        return m_output += (input - m_output) * m_e_pow;
    }

    /// @brief Get the filter output
    [[nodiscard]] inline constexpr T getOutput() const
    {
        return m_output;
    }

    /// @brief Get the filter output
    [[nodiscard]] inline constexpr const T& getOutput() const
    {
        return m_output;
    }
};

}