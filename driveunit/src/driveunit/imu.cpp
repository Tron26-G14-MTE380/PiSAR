#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/logging.h"
#include "pisar/utilities/circular_queue.h"
#include "pisar/driveunit_interface/interface.h"

#include <vector>
#include <FS.h>
#include <LittleFS.h>

using namespace std::chrono_literals;

namespace pisar::driveunit
{

Imu::Imu(
    SPIClassRP2040 &spi, 
    uint8_t cs_pin, uint8_t rx_pin, uint8_t tx_pin, uint8_t sck_pin, std::optional<uint8_t> int1_pin,
    std::string_view calibration_data_file_path, 
    uint16_t sample_rate, uint32_t spi_speed
) :
    m_spi(spi),
    m_cs_pin(cs_pin),
    m_rx_pin(rx_pin),
    m_tx_pin(tx_pin),
    m_sck_pin(sck_pin),
    m_int1_pin(int1_pin),
    m_calibration_data_file_path(calibration_data_file_path),
    m_sample_rate(sample_rate),
    m_sample_time(static_cast<int64_t>(1E6f / sample_rate)),
    m_imu(&spi, cs_pin, spi_speed),
    m_interrupt_callback(std::nullopt)
{}

bool Imu::initialize()
{

    if (m_spi.setRX(m_rx_pin) == false)
    {
        PISAR_LOG_ERROR("Failed to setup RX pin!");
        return false;
    }

    if (m_spi.setTX(m_tx_pin) == false)
    {
        PISAR_LOG_ERROR("Failed to setup TX pin!");
        return false;
    }

    if (m_spi.setSCK(m_sck_pin) == false)
    {
        PISAR_LOG_ERROR("Failed to setup SCK pin!");
        return false;
    }

    if (m_spi.setCS(m_cs_pin) == false)
    {
        PISAR_LOG_ERROR("Failed to setup CS pin!");
        return false;
    }

    // no return for this guy :p
    m_spi.begin();

    if (m_imu.begin() != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to initialize LSM6DSO via SPI!");
        return false;
    }
    
    if (connectionCheck() == false)
    {
        PISAR_LOG_ERROR("Failed to connect to the IMU! Try connecting again!");
        return false;
    }

    // Load calibration data
    if (calibrationDataSaved())
    {
        PISAR_LOG_INFO("Calibration data saved to %s, loading...", m_calibration_data_file_path.data());
        if (!loadCalibrationData())
        {
            PISAR_LOG_ERROR("Failed to load calibration data!");
            return false;
        }
        PISAR_LOG_INFO("Calibration data successfully loaded!");
    }
    else
    {
        PISAR_LOG_WARN("No calibration data found at %s, call calibrate to calibrate IMU", m_calibration_data_file_path.data());
        // Fresh calibration
        m_calibration_data = {
            .accel_offset = {0, 0, 0},
            .gyro_offset = {0, 0, 0}
        };
    }
    
    if (m_imu.Enable_X() != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to enable the IMU's accelerometer!");
        return false;
    }

    if (m_imu.Set_X_ODR(m_sample_rate) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set accelerometer data rate!");
        return false;
    }

    if (m_imu.Set_X_FS(4) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set accelerometer range!");
        return false;
    }

    if (m_imu.Enable_G() != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to enable the IMU's gyro!");
        return false;
    }

    if (m_imu.Set_G_ODR(m_sample_rate) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set gyroscope data rate!");
        return false;
    }

    if (m_imu.Set_G_FS(1000) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set gyroscope range!");
        return false;
    }

    // Setup onboard fifo
    if (m_imu.Set_FIFO_Mode(LSM6DSO_STREAM_MODE) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set IMU fifo mode!");
        return false;
    }

    if (m_imu.Set_FIFO_X_BDR(m_sample_rate) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set accelerometer FIFO batch rate!");
        return false;
    }

    if (m_imu.Set_FIFO_G_BDR(m_sample_rate) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set gryoscope FIFO batch rate!");
        return false;
    }

    return true;
}

[[nodiscard]] bool Imu::setFifoWatermarkInterrupt(const uint8_t num_samples, const InterruptCallback callback)
{
    if (!m_int1_pin.has_value())
    {
        PISAR_LOG_ERROR("IMU interrupt pin not set!");
        return false;
    }

    if (!callback)
    {
        PISAR_LOG_ERROR("Interrupt callback is null!");
        return false;
    }

    // Need to double since watermark level is for gyro and accel samples combined.
    if (m_imu.Set_FIFO_Watermark_Level(num_samples * 2) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set FIFO watermark level!");
        return false;
    }

    lsm6dso_reg_t reg = { .byte = 0 };
    reg.int1_ctrl.int1_fifo_th = 1;

    if (m_imu.Write_Reg(LSM6DSO_INT1_CTRL, reg.byte) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set FIFO watermark interrupt!");
        return false;
    }

    m_interrupt_callback = callback;

    // Setup interrupt
    pinMode(m_int1_pin.value(), INPUT);
    attachInterrupt(digitalPinToInterrupt(m_int1_pin.value()), interruptHandler1, RISING, this);

    return true;
}

bool Imu::calibrate(size_t num_samples, bool save)
{
    setCalibration({
        .accel_offset = {0, 0, 0},
        .gyro_offset = {0, 0, 0}
    });

    constexpr int kDelaySeconds = 3;
    PISAR_LOG_INFO("Starting IMU calibration in %d seconds... Keep the IMU **completely still**!", kDelaySeconds);
    delay(kDelaySeconds * 1000);
    PISAR_LOG_INFO("Starting Calibration...");

    Eigen::Vector3<int32_t> accel_sum(0, 0, 0);
    Eigen::Vector3<int32_t> gyro_sum(0, 0, 0);

    for (size_t i = 0; i < num_samples; i++)
    {
        AccelDataRaw accelDataRaw = readAccelRaw();
        GyroDataRaw gyroDataRaw = readGyroRaw();

        if (accelDataRaw.values.allFinite() && gyroDataRaw.values.allFinite()) // Prevent accumulating bad data
        {
            accel_sum += accelDataRaw.values.cast<int32_t>();
            gyro_sum += gyroDataRaw.values.cast<int32_t>();
        }
        else
        {
            PISAR_LOG_ERROR("Invalid IMU data detected during calibration! Try again and ensure good connections!");
            return false;
        }

        delay(std::chrono::duration_cast<std::chrono::milliseconds>(m_sample_time).count()); // Convert to milliseconds
    }

    // Compute average offsets
    m_calibration_data.accel_offset = (accel_sum / static_cast<int32_t>(num_samples)).cast<int16_t>();
    m_calibration_data.gyro_offset = (gyro_sum / static_cast<int32_t>(num_samples)).cast<int16_t>();

    PISAR_LOG_INFO("IMU Calibration complete.");
    PISAR_LOG_INFO("Accel Offset: x=%i, y=%i, z=%i", m_calibration_data.accel_offset.x(), m_calibration_data.accel_offset.y(), m_calibration_data.accel_offset.z());
    PISAR_LOG_INFO("Gyro Offset: x=%i, y=%i, z=%i", m_calibration_data.gyro_offset.x(), m_calibration_data.gyro_offset.y(), m_calibration_data.gyro_offset.z());

    if (save)
    {
        if (!saveCalibrationData())
        {
            PISAR_LOG_ERROR("Failed to save calibration data.");
            return false;
        }

        PISAR_LOG_INFO("Calibration data saved successfully!");
    }

    return true;
}

[[nodiscard]] size_t Imu::readFifoRaw(const std::span<std::variant<AccelDataRaw, GyroDataRaw>>& output)
{
    const size_t samples_available = fifoSamplesAvailable();
    if (samples_available == 0)
    {
        return 0; // No new data
    }

    const size_t total_samples_to_read = std::min(samples_available, output.size());

    size_t samples_read = 0;

    // Read all available FIFO samples
    for (int i = 0; i < total_samples_to_read; ++i)
    {
        uint8_t tag = 0;
        
        if(m_imu.Get_FIFO_Tag(&tag) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO tag");
            return 0;
        }

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                output[samples_read++] = getAccelDataFifoRaw();
                break;
            case LSM6DSO_GYRO_NC_TAG:
                output[samples_read++] = getGyroDataFifoRaw();
                break;
            default:
                PISAR_LOG_ERROR("Unknown IMU FIFO tag: %u", tag);
                break;
        }
    }

    return samples_read;
}

[[nodiscard]] size_t Imu::readFifo(const std::span<std::variant<AccelData, GyroData>>& output)
{
    const size_t samples_available = fifoSamplesAvailable();
    if (samples_available == 0)
    {
        return 0; // No new data
    }

    const size_t total_samples_to_read = std::min(samples_available, output.size());

    size_t samples_read = 0;

    // Read all available FIFO samples
    for (int i = 0; i < total_samples_to_read; ++i)
    {
        uint8_t tag = 0;

        if(m_imu.Get_FIFO_Tag(&tag) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO tag");
            return 0;
        }

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                output[samples_read++] = getAccelDataFifo();
                break;
            case LSM6DSO_GYRO_NC_TAG:
                output[samples_read++] = getGyroDataFifo();
                break;
            default:
                PISAR_LOG_ERROR("Unknown IMU FIFO tag: %u", tag);
                break;
        }
    }

    return samples_read;
}

[[nodiscard]] size_t Imu::readFifoPaired(const std::span<Data>& output)
{
    size_t samples_available = fifoSamplesAvailable();
    if (samples_available == 0)
    {
        return 0; // No new data
    }

    size_t data_samples = 0;

    CircularQueue<std::pair<AccelData, std::chrono::microseconds>, 32> accel_buffer;
    CircularQueue<std::pair<GyroData, std::chrono::microseconds>, 32> gyro_buffer;

    std::chrono::microseconds timestamp = 0ms;

    int num_x = 0;
    int num_g = 0;

    // Read all available FIFO samples
    while((data_samples + accel_buffer.size() + gyro_buffer.size()) < output.size() && samples_available)
    {
        uint8_t tag = 0;

        if(m_imu.Get_FIFO_Tag(&tag) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO tag");
            return 0;
        }

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                num_x++;
                accel_buffer.push({getAccelDataFifo(), timestamp});
                break;
            case LSM6DSO_GYRO_NC_TAG:
                num_g++;
                gyro_buffer.push({getGyroDataFifo(), timestamp});
                break;
            default:
                PISAR_LOG_ERROR("Unknown IMU FIFO tag: %u", tag);
                break;
        }

        samples_available--;
        timestamp += m_sample_time;

        // Try to pair samples based on timestamps
        while (!accel_buffer.empty() && !gyro_buffer.empty())
        {
            auto [accel, accel_time] = accel_buffer.front();
            auto [gyro, gyro_time] = gyro_buffer.front();

            if (accel_time == gyro_time) // Perfect match
            {
                output[data_samples++] = Data{accel, gyro};
                accel_buffer.pop();
                gyro_buffer.pop();
            }
            else if (accel_time < gyro_time) // Accel is ahead, pair with last known gyro
            {
                output[data_samples++] = Data{accel, gyro_buffer.front().first};
                accel_buffer.pop();
            }
            else // Gyro is ahead, pair with last known accel
            {
                output[data_samples++] = Data{accel_buffer.front().first, gyro};
                gyro_buffer.pop();
            }
        }
    }

    // Handle remaining unmatched samples (use last known values)
    while (!accel_buffer.empty() && data_samples < output.size())
    {
        GyroData gyro_data;
        if (data_samples == 0)
        {
            PISAR_LOG_WARN("No gyro data available to pair with remaining accel data!");
            gyro_data = {};
        }
        else
        {
            gyro_data = output[data_samples - 1].gyro_data;
        }

        output[data_samples++] = Data{accel_buffer.front().first, gyro_data};
        accel_buffer.pop();
    }

    while (!gyro_buffer.empty() && data_samples < output.size())
    {
        AccelData accel_data;
        if (data_samples == 0)
        {
            PISAR_LOG_WARN("No accel data available to pair with remaining gyro data!");
            accel_data = {};
        }
        else
        {
            accel_data = output[data_samples - 1].accel_data;
        }

        output[data_samples++] = Data{accel_data, gyro_buffer.front().first};
        gyro_buffer.pop();
    }
    return data_samples;
}

[[nodiscard]] std::vector<Imu::Data> Imu::readFifoBuffered()
{
    const size_t samples_available = fifoSamplesAvailable();
    if (samples_available == 0)
    {
        return {}; // Return empty vector if no data
    }

    std::vector<Data> fifo_data(samples_available);

    // Call the fixed-size version to fill the vector
    size_t samples_read = readFifoPaired(std::span(fifo_data));

    // Resize vector to match actual samples read
    fifo_data.resize(samples_read);

    return fifo_data;
}

} // namespace pisar::driveunit
