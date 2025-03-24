#pragma once

#include <Eigen/Dense>

#include <chrono>

namespace pisar::mcp {

/**
 * @brief Extended Kalman Filter (EKF) with Constant Acceleration Model for N-dimensional trajectories.
 * @tparam N Dimensionality of the state vector (e.g., 2 for 2D, 3 for 3D, etc.)
 */
template <int N>
class ExtendedKalmanFilter {
private:
    Eigen::Matrix<double, 3 * N, 1> m_state;               ///< State vector [pos, vel, acc]
    Eigen::Matrix<double, 3 * N, 3 * N> m_covariance;      ///< Estimate covariance (P)
    Eigen::Matrix<double, 3 * N, 3 * N> m_process_noise;   ///< Process noise covariance (Q)
    Eigen::Matrix<double, N, N> m_measurement_noise;      ///< Measurement noise covariance (R)

public:
    ExtendedKalmanFilter()
    {
        m_state.setZero();
        m_covariance = Eigen::Matrix<double, 3 * N, 3 * N>::Identity();
        m_process_noise = Eigen::Matrix<double, 3 * N, 3 * N>::Identity() * 0.01;
        m_measurement_noise = Eigen::Matrix<double, N, N>::Identity() * 1.0;
    }

    /**
     * @brief Predicts the next state using a constant acceleration model.
     * @param delta_t Time step since the last update.
     */
    void predict(std::chrono::duration<float> delta_t)
    {
        Eigen::Matrix<double, 3 * N, 1> new_state = nonlinearMotionModel(m_state, delta_t.count());

        // Compute Jacobian of the motion model
        Eigen::Matrix<double, 3 * N, 3 * N> Jf = computeJacobian(delta_t.count());

        // Predict state and update covariance
        m_state = new_state;
        m_covariance = Jf * m_covariance * Jf.transpose() + m_process_noise;
    }

    /**
     * @brief Updates the state using a new measurement.
     * @param measurement Observed [x, y, ...] position.
     */
    void update(const Eigen::Matrix<double, N, 1>& measurement) {
        // Compute measurement residual
        Eigen::Matrix<double, N, 1> innovation = measurement - getPredictedMeasurement();

        // Compute Jacobian of the measurement function
        Eigen::Matrix<double, N, 3 * N> Jh = computeMeasurementJacobian();

        // Compute Kalman gain
        Eigen::Matrix<double, N, N> innovation_covariance = Jh * m_covariance * Jh.transpose() + m_measurement_noise;
        Eigen::Matrix<double, 3 * N, N> kalman_gain = m_covariance * Jh.transpose() * innovation_covariance.inverse();

        // Update state and covariance
        m_state += kalman_gain * innovation;
        m_covariance = (Eigen::Matrix<double, 3 * N, 3 * N>::Identity() - kalman_gain * Jh) * m_covariance;
    }

    /**
     * @brief Returns the filtered position.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 1> getFilteredPosition() const {
        return m_state.template block<N, 1>(0, 0); // Extract position elements
    }

    /**
     * @brief Returns the filtered velocity.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 1> getFilteredVelocity() const {
        return m_state.template block<N, 1>(N, 0); // Extract velocity elements
    }

    /**
     * @brief Returns the filtered acceleration.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 1> getFilteredAcceleration() const {
        return m_state.template block<N, 1>(2 * N, 0); // Extract acceleration elements
    }

private:
    /**
     * @brief Defines the constant acceleration motion model.
     */
    [[nodiscard]] Eigen::Matrix<double, 3 * N, 1> nonlinearMotionModel(
        const Eigen::Matrix<double, 3 * N, 1>& state, double delta_t) const
    {
        Eigen::Matrix<double, 3 * N, 1> new_state = state;
        for (int i = 0; i < N; ++i) {
            new_state(i) += state(N + i) * delta_t + 0.5 * state(2 * N + i) * delta_t * delta_t; // Position update
            new_state(N + i) += state(2 * N + i) * delta_t; // Velocity update
        }
        return new_state;
    }

    /**
     * @brief Computes the Jacobian matrix of the motion model.
     */
    [[nodiscard]] Eigen::Matrix<double, 3 * N, 3 * N> computeJacobian(double delta_t) const
    {
        Eigen::Matrix<double, 3 * N, 3 * N> Jf = Eigen::Matrix<double, 3 * N, 3 * N>::Identity();
        for (int i = 0; i < N; ++i) {
            Jf(i, N + i) = delta_t;      // Partial derivative of position w.r.t velocity
            Jf(i, 2 * N + i) = 0.5 * delta_t * delta_t; // Partial derivative of position w.r.t acceleration
            Jf(N + i, 2 * N + i) = delta_t; // Partial derivative of velocity w.r.t acceleration
        }
        return Jf;
    }

    /**
     * @brief Computes the expected measurement from the current state.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 1> getPredictedMeasurement() const {
        return m_state.template block<N, 1>(0, 0); // Measurement is position only
    }

    /**
     * @brief Computes the Jacobian matrix of the measurement function.
     */
    [[nodiscard]] Eigen::Matrix<double, N, 3 * N> computeMeasurementJacobian() const {
        Eigen::Matrix<double, N, 3 * N> Jh = Eigen::Matrix<double, N, 3 * N>::Zero();
        for (int i = 0; i < N; ++i) {
            Jh(i, i) = 1.0; // Partial derivative of measurement w.r.t position
        }
        return Jh;
    }
};

}