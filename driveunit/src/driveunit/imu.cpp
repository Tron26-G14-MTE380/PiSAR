#include "pisar/driveunit/imu.h"
#include "pisar/driveunit/logging.h"
#include "pisar/utilities/circular_queue.h"

#include <vector>

using namespace std::chrono_literals;

namespace pisar::driveunit
{

Imu::Imu(SPIClassRP2040 &spi, uint8_t cs_pin, uint8_t rx_pin, uint8_t tx_pin, uint8_t sck_pin, uint16_t sample_rate, uint32_t spi_speed) :
    m_sample_rate(sample_rate),
    m_sample_time(static_cast<std::chrono::microseconds::rep>(1E6f / sample_rate)),
    m_imu(&spi, cs_pin, spi_speed),
    m_spi(spi),
    m_cs_pin(cs_pin),
    m_rx_pin(rx_pin),
    m_tx_pin(tx_pin),
    m_sck_pin(sck_pin)
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
        PISAR_LOG_ERROR("Failed to connect! Try connecting again!");
        return false;
    }

    if (m_imu.Set_X_FS(2) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set accelerometer range!");
        return false;
    }

    if (m_imu.Set_X_ODR(m_sample_rate) != LSM6DSO_OK) // Maps to 833 Hz
    {
        PISAR_LOG_ERROR("Failed to set accelerometer data rate!");
        return false;
    }

    if (m_imu.Set_G_FS(125) != LSM6DSO_OK)
    {
        PISAR_LOG_ERROR("Failed to set gyroscope range!");
        return false;
    }

    if (m_imu.Set_G_ODR(m_sample_rate) != LSM6DSO_OK) // Maps to 833 Hz
    {
        PISAR_LOG_ERROR("Failed to set gyroscope data rate!");
        return false;
    }

    // // Setup onboard fifo
    // if (m_imu.Set_FIFO_Mode(LSM6DSO_STREAM_MODE) != LSM6DSO_OK)
    // {
    //     PISAR_LOG_ERROR("Failed to set IMU fifo mode!");
    //     return false;
    // }

    // if (m_imu.Set_FIFO_X_BDR(m_sample_rate) != LSM6DSO_OK)
    // {
    //     PISAR_LOG_ERROR("Failed to set accelerometer FIFO batch rate!");
    //     return false;
    // }

    // if (m_imu.Set_FIFO_G_BDR(m_sample_rate) != LSM6DSO_OK)
    // {
    //     PISAR_LOG_ERROR("Failed to set gryoscope FIFO batch rate!");
    //     return false;
    // }

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
        uint8_t data[6];

        if(m_imu.Get_FIFO_Tag(&tag) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO tag");
            return 0;
        }

        if(m_imu.Get_FIFO_Data(data) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO data");
            return 0;
        }

        const Eigen::Vector3<DataRawValueT> raw_data {
            ((int16_t)data[1] << 8) | data[0],
            ((int16_t)data[3] << 8) | data[2],
            ((int16_t)data[5] << 8) | data[4]
        };

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                output[samples_read++].emplace<AccelDataRaw>(raw_data);
                break;
            case LSM6DSO_GYRO_NC_TAG:
                output[samples_read++].emplace<GyroDataRaw>(raw_data);
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
        Eigen::Vector3<DataValueT> data;

        if(m_imu.Get_FIFO_Tag(&tag) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO tag");
            return 0;
        }

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                if (m_imu.Get_FIFO_X_Axes(data.data()) != LSM6DSO_OK)
                {
                    PISAR_LOG_ERROR("Failed to get FIFO accelerometer data");
                    return 0;
                }
                output[samples_read++].emplace<AccelData>(data);
                break;
            case LSM6DSO_GYRO_NC_TAG:
                if (m_imu.Get_FIFO_G_Axes(data.data()) != LSM6DSO_OK)
                {
                    PISAR_LOG_ERROR("Failed to get FIFO gyro data");
                    return 0;
                }
                output[samples_read++].emplace<GyroData>(data);
                break;
            default:
                PISAR_LOG_ERROR("Unknown IMU FIFO tag: %u", tag);
                break;
        }
    }

    return samples_read;
}

[[nodiscard]] size_t Imu::readFifo(const std::span<Data>& output)
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

    // Read all available FIFO samples
    while((data_samples + accel_buffer.size() + gyro_buffer.size()) < output.size() && samples_available)
    {
        uint8_t tag = 0;
        Eigen::Vector3<DataValueT> data;

        if(m_imu.Get_FIFO_Tag(&tag) != LSM6DSO_OK)
        {
            PISAR_LOG_ERROR("Failed to get FIFO tag");
            return 0;
        }

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                if (m_imu.Get_FIFO_X_Axes(data.data()) != LSM6DSO_OK)
                {
                    PISAR_LOG_ERROR("Failed to get FIFO accelerometer data");
                    return 0;
                }
                accel_buffer.push({AccelData{data}, timestamp});
                break;
            case LSM6DSO_GYRO_NC_TAG:
                if (m_imu.Get_FIFO_G_Axes(data.data()) != LSM6DSO_OK)
                {
                    PISAR_LOG_ERROR("Failed to get FIFO gyro data");
                    return 0;
                }
                gyro_buffer.push({GyroData{data}, timestamp});
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
        output[data_samples++] = Data{accel_buffer.front().first, gyro_buffer.empty() ? GyroData{} : gyro_buffer.back().first};
        accel_buffer.pop();
    }

    while (!gyro_buffer.empty() && data_samples < output.size())
    {
        output[data_samples++] = Data{accel_buffer.empty() ? AccelData{} : accel_buffer.back().first, gyro_buffer.front().first};
        gyro_buffer.pop();
    }
    return data_samples;
}

[[nodiscard]] std::vector<Imu::Data> Imu::readFifo()
{
    const size_t samples_available = fifoSamplesAvailable();
    if (samples_available == 0)
    {
        return {}; // Return empty vector if no data
    }

    std::vector<Data> fifo_data(samples_available);

    // Call the fixed-size version to fill the vector
    size_t samples_read = readFifo(std::span(fifo_data));

    // Resize vector to match actual samples read
    fifo_data.resize(samples_read);

    return fifo_data;
}

} // namespace pisar::driveunit
