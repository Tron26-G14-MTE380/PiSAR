#pragma once

#include "pisar/driveunit/logging.h"

#include <Eigen/Dense>
#include <zpp_bits.h>

#include <SPI.h>
#include <LSM6DSOSensor.h>
#include <LittleFS.h>

#include <cstdint>
#include <span>
#include <variant>
#include <vector>
#include <chrono>

namespace pisar::driveunit
{

/**
 * @brief Provides an interface for a 6-axis Imu sensor using SPI.
 */
class Imu
{
public:

    using DataRawValueT = int16_t;
    using DataValueT = float;

    struct AccelDataRaw { Eigen::Vector3<DataRawValueT> values; };
    struct AccelData { Eigen::Vector3<DataValueT> values; };
    struct GyroDataRaw { Eigen::Vector3<DataRawValueT> values; };
    struct GyroData { Eigen::Vector3<DataValueT> values; };
    struct TestDataAccel { Eigen::Vector3<int32_t> values; };
    struct TestDataGyro { Eigen::Vector3<int32_t> values; };

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

    /**
     * @brief Stores calibration offsets for accelerometer and gyroscope.
     */
    struct CalibrationData
    {
        using serialize = zpp::bits::members<2>;

        Eigen::Vector3<DataRawValueT> accel_offset; ///< Accelerometer offset values
        Eigen::Vector3<DataRawValueT> gyro_offset;  ///< Gyroscope offset values

        /**
         * @brief Saves IMU calibration data to LittleFS.
         * @param file_path The location to save to.
         * @return True if the data was successfully saved, false otherwise.
         */
        [[nodiscard]] bool save(std::string_view file_path) const;

        /**
         * @brief Loads IMU calibration data from LittleFS.
         * @param file_path The location to load from.
         * @return True if the data was successfully loaded, false otherwise.
         */
        [[nodiscard]] bool load(std::string_view file_path);
    };

    using InterruptCallback = std::function<void()>;

private:

    SPIClassRP2040 &m_spi;                                  ///< SPI bus reference.
    uint8_t m_cs_pin;                                       ///< Chip Select (CS) pin.
    uint8_t m_rx_pin;                                       ///< RX (MISO) pin.
    uint8_t m_tx_pin;                                       ///< TX (MOSI) pin.
    uint8_t m_sck_pin;                                      ///< Clock (SCK) pin.
    std::optional<uint8_t> m_int1_pin;                      ///< INT1 pin.
    std::string_view m_calibration_data_file_path;          ///< Calibration data file path.
    uint16_t m_sample_rate;                                 ///< Sample rate.
    std::chrono::microseconds m_sample_time;                ///< Sample time.
    LSM6DSOSensor m_imu;                                    ///< Underlying imu sensor driver.

    CalibrationData m_calibration_data;                     ///< Calibration data.

    std::optional<InterruptCallback> m_interrupt_callback;  ///< Interrupt callback.

public:

    /**
     * @brief Constructs an Imu object.
     * @param spi Reference to the SPI bus (RP2040).
     * @param cs_pin Chip select (CS) pin for SPI communication.
     * @param rx_pin The SPI RX pin.
     * @param tx_pin The SPI TX pin.
     * @param sck_pin The SPI SCK pin.
     * @param int1_pin The interrupt pin for the IMU.
     * @param calibration_data_file_path The path to the calibration data file.
     * @param sample_rate The accelerometer and gyroscope sample rate (default: 800).
     * @param spi_speed The SPI communication speed in Hz (default: 1000000).
     */
    Imu(
        SPIClassRP2040 &spi, 
        uint8_t cs_pin, uint8_t rx_pin, uint8_t tx_pin, uint8_t sck_pin, std::optional<uint8_t> int1_pin,
        std::string_view calibration_data_file_path, 
        uint16_t sample_rate = 833, uint32_t spi_speed = 1'000'000
    );

    /// @brief Destructor
    ~Imu();

    [[nodiscard]] inline uint16_t getSampleRate() { return m_sample_rate; }
    [[nodiscard]] inline std::chrono::microseconds getSampleTime() { return m_sample_time; }

    /**
     * @brief Initializes the IMU, setting up SPI and sensor configurations.
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Set an interrupt on reaching FIFO watermark threshold.
     * @param num_samples Number of data samples to generate on (number of accel + gyro data pairs).
     * @param callback The interrupt callback function.
     */
    [[nodiscard]] bool setFifoWatermarkInterrupt(const uint8_t num_samples, const InterruptCallback callback);

    /// @brief Returns whether data is ready.
    [[nodiscard]] inline bool accelDataReady()
    {
        uint8_t ready = false;
        if (m_imu.Get_X_DRDY_Status(&ready) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get accelerometer ready status!");
            return false;
        }

        PISAR_LOG_ERROR("ready value: %d", ready);

        return ready;
    }

    /// @brief Returns whether data is ready.
    [[nodiscard]] inline bool gyroDataReady()
    {
        uint8_t ready = false;
        if (m_imu.Get_G_DRDY_Status(&ready) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get gyroscope ready status!");
            return false;
        }
        return ready;
    }

    /**
     * @brief Reads the latest raw accelerometer data.
     * @return The accelerometer x, y, z readings.
     */
    [[nodiscard]] inline AccelDataRaw readAccelRaw()
    {
        AccelDataRaw data;
        if (m_imu.Get_X_AxesRaw(data.values.data()) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to read raw accelerometer data");
            return {};
        }

        data.values -= m_calibration_data.accel_offset;

        return data;
    }

    /**
     * @brief Reads the latest accelerometer data.
     * @return The accelerometer x, y, z readings.
     * @note [m/s^2]
     */
    [[nodiscard]] inline AccelData readAccel()
    {
        const AccelDataRaw rawData = readAccelRaw();
        // TODO error check sensitivity
        return AccelData {.values = accelSensitivityAdjustment<Eigen::Vector3f>(rawData.values.cast<float>()).value()};
    }

    /**
     * @brief Reads the latest raw gryoscope data.
     * @return The gyroscope x, y, z readings.
     */
    [[nodiscard]] inline GyroDataRaw readGyroRaw()
    {
        GyroDataRaw data;
        if (m_imu.Get_G_AxesRaw(data.values.data()) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to read raw gyroscope data");
            return {};
        }

        data.values -= m_calibration_data.gyro_offset;

        return data;
    }

    /**
     * @brief Reads the latest gryoscope data.
     * @return The gyroscope x, y, z readings.
     * @note [deg/s]
     */
    [[nodiscard]] inline GyroData readGyro()
    {
        const GyroDataRaw rawData = readGyroRaw();
        // TODO error check sensitivity
        return GyroData {.values = gyroSensitivityAdjustment<Eigen::Vector3f>(rawData.values.cast<float>()).value()};
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
        if (m_imu.Get_FIFO_Num_Samples(&samples_available) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed get number of FIFO samples available.");
            return {};
        }
        return samples_available;
    }

    /**
     * @brief Reads the WHO_AM_I register to verify IMU communication.
     * @return The expected ID (0x6A) if successful, otherwise 0x00 or 0xFF if communication fails.
     */
    [[nodiscard]] inline std::optional<uint8_t> readWhoAmI()
    {
        uint8_t who_am_i = 0;
        if (m_imu.ReadID(&who_am_i) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to read WHO_AM_I register");
            return std::nullopt;
        }
        return who_am_i;
    }


    /**
     * @brief Checks the connection to the IMU by reading the WHO_AM_I register.
     * @return True if the WHO_AM_I register value matches the expected ID (0x6C), false otherwise.
     */
    [[nodiscard]] inline bool connectionCheck()
    {
        const auto whoami = readWhoAmI();
        if (!whoami)
        {
            return false;        
        }

        return whoami.value() == 0x6C;
    }

    /**
     * @brief Gets the accelerometer sensitivity.
     * @return The accelerometer sensitivity if successful, otherwise -1.0f to indicate failure.
     */
    [[nodiscard]] inline std::optional<float> getAccelSensitivity()
    {
        float sensitivity = 0;
        if (m_imu.Get_X_Sensitivity(&sensitivity) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get accelerometer sensitivity.");
            return std::nullopt;
        }

        return sensitivity / 100.0;
    }

    /**
     * @brief Gets the gyroscope sensitivity.
     * @return The gyroscope sensitivity if successful, otherwise -1.0f to indicate failure.
     */
    [[nodiscard]] inline std::optional<float> getGyroSensitivity()
    {
        float sensitivity = 0.0;
        if (m_imu.Get_G_Sensitivity(&sensitivity) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get gyro sensitivity.");
            return std::nullopt;
        }

        return sensitivity / 1000.0f;
    }

    /**
     * @brief Gets the accelerometer sensitivity.
     * @return The accelerometera sensitivity if successful, otherwise -1.0f to indicate failure.
     */
    template<typename T>
    [[nodiscard]] inline std::optional<T> accelSensitivityAdjustment(const T&& value)
    {
        auto sensitivity = getAccelSensitivity();
        if (!sensitivity)
        {
            return std::nullopt;
        }

        return T(value * sensitivity.value());
    }

    /**
     * @brief Gets the gyroscope sensitivity.
     * @return The gyroscope sensitivity if successful, otherwise -1.0f to indicate failure.
     */
    template<typename T>
    [[nodiscard]] inline std::optional<T> gyroSensitivityAdjustment(const T&& value)
    {
        auto sensitivity = getGyroSensitivity();
        if (!sensitivity)
        {
            return std::nullopt;
        }

        return T(value * sensitivity.value());
    }

    /**
     * @brief Calibrates the IMU by computing offsets for the accelerometer and gyroscope.
     *
     * @param num_samples The number of samples to collect for calibration.
     * @param save Whether to save the calibration data to the filesystem.
     * @return True if calibration (and saving) was successful, false otherwise.
     *
     * @note Ensure the IMU is motionless during calibration to get accurate offsets.
     * @note Offsets are stored as integer values to match the IMU's processed data format.
     * @note Max number of samples is 65'535.
     */
    bool calibrate(size_t num_samples, bool save = true);

    /**
     * @brief Sets the calibration data.
     * @param calib_data The calibration data to set.
     */
    inline void setCalibration(const CalibrationData& calib_data)
    {
        m_calibration_data = calib_data;
    }

    /// @brief Gets the calibration data.
    [[nodiscard]] inline CalibrationData getCalibration() const
    {
        return m_calibration_data;
    }

    /// @brief Get whether the calibration data has been saved.
    [[nodiscard]] inline bool calibrationDataSaved() const
    {
        return LittleFS.exists(m_calibration_data_file_path.data());
    }

    /**
     * @brief Loads the calibration data from the filesystem.
     * @return True if the data was successfully loaded, false otherwise.
     */
    [[nodiscard]] inline bool loadCalibrationData()
    {
        return m_calibration_data.load(m_calibration_data_file_path);
    }

    /**
     * @brief Saves the calibration data to the filesystem.
     * @return True if the data was successfully saved, false otherwise.
     */
    [[nodiscard]] inline bool saveCalibrationData() const
    {
        return m_calibration_data.save(m_calibration_data_file_path);
    }

    /**
     * @brief Reads raw accelerometer and gyroscope samples from the IMU FIFO buffer. 
     * @param output Output buffer to store the raw samples.
     * @return The number of samples successfully read from the FIFO.
     */
    [[nodiscard]] size_t readFifoRaw(const std::span<std::variant<AccelDataRaw, GyroDataRaw>>& output);

    /**
     * @brief Reads raw accelerometer and gyroscope samples from the FIFO and applies scaling.
     * @param output Output buffer to store the scaled IMU samples.
     * @return The number of samples successfully read from the FIFO.
     */
    [[nodiscard]] size_t readFifo(const std::span<std::variant<AccelData, GyroData>>& output);

    /**
     * @brief Reads raw FIFO samples, pairs accelerometer and gyroscope readings, applies scaling, and stores them in order. 
     * @param output Output buffer to store paired IMU samples as `Data` structures.
     * @return The number of valid accelerometer-gyroscope sample pairs stored in the output.
     */
    [[nodiscard]] size_t readFifoPaired(const std::span<Data>& output);

    /**
     * @brief Reads and buffers FIFO samples into a dynamically sized vector.
     * @return A `std::vector<Data>` containing synchronized accelerometer-gyroscope pairs.
     */
    [[nodiscard]] std::vector<Data> readFifoBuffered();

private:
    /**
     * @brief Gets the raw data from the FIFO buffer.
     */
    [[nodiscard]] inline Eigen::Vector3<DataRawValueT> getDataFifoRaw()
    {
        uint8_t data[6];

        if(m_imu.Get_FIFO_Data(data) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO data");
            return {};
        }

        return Eigen::Vector3<DataRawValueT> {
            static_cast<DataRawValueT>(((int16_t)data[1] << 8) | data[0]),
            static_cast<DataRawValueT>(((int16_t)data[3] << 8) | data[2]),
            static_cast<DataRawValueT>(((int16_t)data[5] << 8) | data[4])
        };
    }

    /**
     * @brief Gets the raw accelerometer data from the FIFO buffer.
     */
    [[nodiscard]] inline AccelDataRaw getAccelDataFifoRaw()
    {
        return AccelDataRaw { .values = getDataFifoRaw() - m_calibration_data.accel_offset };
    }

    /**
     * @brief Gets the accelerometer data from the FIFO buffer.
     */
    [[nodiscard]] inline AccelData getAccelDataFifo()
    {
        const AccelDataRaw rawData = getAccelDataFifoRaw();
        // TODO error check sensitivity
        return AccelData {.values = accelSensitivityAdjustment<Eigen::Vector3f>(rawData.values.cast<float>()).value()};
    }

    /**
     * @brief Gets the raw gyroscope data from the FIFO buffer.
     */
    [[nodiscard]] inline GyroDataRaw getGyroDataFifoRaw()
    {
        return GyroDataRaw { .values = getDataFifoRaw() - m_calibration_data.gyro_offset };
    }

    /**
     * @brief Gets the gyroscope data from the FIFO buffer.
     */
    [[nodiscard]] inline GyroData getGyroDataFifo()
    {
        const GyroDataRaw rawData = getGyroDataFifoRaw();
        // TODO error check sensitivity
        return GyroData {.values = gyroSensitivityAdjustment<Eigen::Vector3f>(rawData.values.cast<float>()).value()};
    }
    
    static inline void interruptHandler1(Imu* p_context)
    {
        if (p_context->m_interrupt_callback)
        {
            p_context->m_interrupt_callback.value()();
        }
    }
};

}