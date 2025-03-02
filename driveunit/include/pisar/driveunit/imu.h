#pragma once

#include <Eigen/Dense>

#include <SPI.h>
#include <LSM6DSOSensor.h>

#include <cstdint>
#include <span>
#include <variant>
#include <vector>

namespace pisar::driveunit
{


/**
 * @brief Provides an interface for a 6-axis Imu sensor using SPI.
 */
class Imu
{
public:

    using DataRawValueT = int16_t;
    using DataValueT = int32_t;

    struct AccelDataRaw { Eigen::Vector3<DataRawValueT> values; };
    struct AccelData { Eigen::Vector3<DataValueT> values; };
    struct GyroDataRaw { Eigen::Vector3<DataRawValueT> values; };
    struct GyroData { Eigen::Vector3<DataValueT> values; };

    struct DataRaw
    {
        AccelDataRaw accel_data;
        GyroDataRaw gyro_data;
    };

    struct Data
    {
        AccelData accel_data;
        GyroData gyro_data;
    };

private:
    uint16_t m_sample_rate;     ///< Sample rate.
    uint32_t m_sample_time_us;  ///< Sample time.
    LSM6DSOSensor m_imu;        ///< Underlying imu sensor driver.

public:

    /**
     * @brief Constructs an Imu object.
     * @param spi Reference to the SPI bus.
     * @param cs_pin Chip select (CS) pin for SPI communication.
     * @param sample_rate The accelerometer and gyroscope sample rate.
     */
    Imu(SPIClass &spi, uint8_t cs_pin, uint16_t sample_rate = 800);

    /// @brief Destructor
    ~Imu();

    [[nodiscard]] inline constexpr uint16_t getSampleRate() { return m_sample_rate; }
    [[nodiscard]] inline constexpr uint32_t getSampleTimeUs() { return m_sample_time_us; }

    /// @brief Initializes the Imu sensor.
    void initialize();

    /// @brief Returns whether data is ready.
    bool accelDataReady()
    {
        uint8_t ready = false;
        m_imu.Get_X_DRDY_Status(&ready); // TODO ERROR CHECK
        return ready;
    }

    /// @brief Returns whether data is ready.
    bool gyroDataReady()
    {
        uint8_t ready = false;
        m_imu.Get_G_DRDY_Status(&ready); // TODO ERROR CHECK
        return ready;
    }

    /**
     * @brief Reads the latest raw accelerometer data.
     * @return The accelerometer x, y, z readings.
     */
    [[nodiscard]] inline AccelDataRaw readAccelRaw()
    {
        AccelDataRaw data;
        m_imu.Get_X_AxesRaw(data.values.data()); // TODO ERROR CHECK
        return data;
    }

    /**
     * @brief Reads the latest accelerometer data.
     * @return The accelerometer x, y, z readings.
     */
    [[nodiscard]] inline AccelData readAccel()
    {
        AccelData data;
        m_imu.Get_X_Axes(data.values.data()); // TODO ERROR CHECK
        return data;
    }

    /**
     * @brief Reads the latest raw gryoscope data.
     * @return The gyroscope x, y, z readings.
     */
    [[nodiscard]] inline GyroDataRaw readGyroRaw()
    {
        GyroDataRaw data;
        m_imu.Get_G_AxesRaw(data.values.data()); // TODO ERROR CHECK
        return data;
    }

    /**
     * @brief Reads the latest gryoscope data.
     * @return The gyroscope x, y, z readings.
     */
    [[nodiscard]] inline GyroData readGyro()
    {
        GyroData data;
        m_imu.Get_G_Axes(data.values.data()); // TODO ERROR CHECK
        return data;
    }

    /**
     * @brief Reads both raw accelerometer and gyroscope data at once.
     * @return The Imu data including the accelerometer and gyroscope data.
     */
    [[nodiscard]] inline DataRaw readRaw()
    {
        return {readAccelRaw(), readGyroRaw()};
    }

    /**
     * @brief Reads both accelerometer and gyroscope data at once.
     * @return The Imu data including the accelerometer and gyroscope data.
     */
    [[nodiscard]] inline Data read()
    {
        return {readAccel(), readGyro()};
    }

    [[nodiscard]] inline uint16_t fifoSamplesAvailable()
    {
        uint16_t samples_available = 0;
        m_imu.Get_FIFO_Num_Samples(&samples_available); // TODO ERROR CHECK
        return samples_available;
    }

    /**
     * @brief Reads the stream of raw accelerometer and gyroscope samples from the fifo.
     * @param output Output buffer to store the samples.
     */
    [[nodiscard]] size_t readFifoRaw(const std::span<std::variant<AccelDataRaw, GyroDataRaw>>& output);

    /**
     * @brief Reads the stream of accelerometer and gyroscope samples from the fifo.
     * @param output Output buffer to store the samples.
     */
    [[nodiscard]] size_t readFifo(const std::span<std::variant<AccelData, GyroData>>& output);

    /**
     * @brief Reads the stream of accelerometer and gyroscope samples from the fifo.
     * @param output Output buffer to store the imu data samples.
     */
    [[nodiscard]] size_t readFifo(const std::span<Data>& output);

    /**
     * @brief Reads the stream of accelerometer and gyroscope samples from the fifo.
     *
     * @return List of accelerometer and gyroscope data pairs.
     */
    [[nodiscard]] std::vector<Data> readFifo();
};

}