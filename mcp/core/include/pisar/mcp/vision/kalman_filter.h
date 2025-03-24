#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace pisar::mcp {

/**
 * @brief Generalized N-Dimensional Kalman Filter for trajectory filtering.
 * @tparam N Dimensionality of the state vector (e.g., 2 for 2D, 3 for 3D, etc.)
 */
template <int N>
class KalmanFilter {
private:
    Eigen::Matrix<double, 2 * N, 2 * N> m_state_transition;   ///< State transition matrix (F)
    Eigen::Matrix<double, 2 * N, N> m_control_matrix;        ///< Control matrix (B) (not used for now)
    Eigen::Matrix<double, 2 * N, 2 * N> m_process_noise;     ///< Process noise covariance (Q)
    Eigen::Matrix<double, N, 2 * N> m_measurement_matrix;    ///< Measurement matrix (H)
    Eigen::Matrix<double, N, N> m_measurement_noise;        ///< Measurement noise covariance (R)
    Eigen::Matrix<double, 2 * N, 2 * N> m_covariance;        ///< Estimate covariance (P)
    Eigen::Matrix<double, 2 * N, 1> m_state;                 ///< State vector [pos1, pos2, ..., vel1, vel2, ...]

public:
    /**
     * @brief Constructor for Kalman filter with default parameters.
     */
    KalmanFilter()
    {
        m_state_transition.setIdentity();
        for (int i = 0; i < N; ++i)
        {
            m_state_transition(i, N + i) = 1.0; // Position update using velocity
        }

        m_measurement_matrix.setZero();
        for (int i = 0; i < N; ++i)
        {
            m_measurement_matrix(i, i) = 1.0; // We only observe position
        }

        m_process_noise = Eigen::Matrix<double, 2 * N, 2 * N>::Identity() * 0.01;
        m_measurement_noise = Eigen::Matrix<double, N, N>::Identity() * 1.0;
        m_covariance = Eigen::Matrix<double, 2 * N, 2 * N>::Identity();
        m_state.setZero();
    }

    /**
     * @brief Predicts the next state based on the current model.
     */
    void predict()
    {
        m_state = m_state_transition * m_state;
        m_covariance = m_state_transition * m_covariance * m_state_transition.transpose() + m_process_noise;
    }

    /**
     * @brief Updates the state with a new trajectory measurement.
     * @param measurement Observed [x, y, ...] position.
     */
    void update(const Eigen::Matrix<double, N, 1>& measurement)
    {
        Eigen::Matrix<double, N, 1> innovation = measurement - (m_measurement_matrix * m_state);
        Eigen::Matrix<double, N, N> innovation_covariance = m_measurement_matrix * m_covariance * m_measurement_matrix.transpose() + m_measurement_noise;
        Eigen::Matrix<double, 2 * N, N> kalman_gain = m_covariance * m_measurement_matrix.transpose() * innovation_covariance.inverse();

        m_state += kalman_gain * innovation;
        m_covariance = (Eigen::Matrix<double, 2 * N, 2 * N>::Identity() - kalman_gain * m_measurement_matrix) * m_covariance;
    }

    /**
     * @brief Returns the filtered position.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 1> getFilteredPosition() const
    {
        return m_state.template block<N, 1>(0, 0); // Extract position elements
    }

    /**
     * @brief Returns the filtered velocity.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 1> getFilteredVelocity() const
    {
        return m_state.template block<N, 1>(N, 0); // Extract velocity elements
    }
};

}