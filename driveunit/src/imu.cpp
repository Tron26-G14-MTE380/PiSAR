#include "pisar/driveunit/imu.h"

#include "pisar/circular_queue.h"

#include <vector>

namespace pisar::driveunit
{


Imu::Imu(SPIClass &spi, uint8_t cs_pin, uint16_t sample_rate) :
    m_sample_rate(sample_rate),
    m_sample_time_us(1'000'000.0f / sample_rate),
    m_imu(&spi, cs_pin)
{
}


void Imu::initialize()
{
    if (m_imu.begin() != LSM6DSO_OK)
    {
        Serial.println("Failed to initialize LSM6DSO via SPI!");
        return;
    }

    if (m_imu.Set_X_FS(2) != LSM6DSO_OK)
    {
        Serial.println("Failed to set accelerometer range!");
        return;
    }

    if (m_imu.Set_X_ODR(m_sample_rate) != LSM6DSO_OK) // Maps to 833 Hz
    {
        Serial.println("Failed to set accelerometer data rate!");
        return;
    }

    if (m_imu.Set_G_FS(125) != LSM6DSO_OK)
    {
        Serial.println("Failed to set gryoscope range!");
        return;
    }

    if (m_imu.Set_G_ODR(m_sample_rate) != LSM6DSO_OK) // Maps to 833 Hz
    {
        Serial.println("Failed to set gyroscope data rate!");
        return;
    }

    // Setup onboard fifo
    if (m_imu.Set_FIFO_Mode(LSM6DSO_STREAM_MODE) != LSM6DSO_OK)
    {
        Serial.println("Failed to set IMU fifo mode!");
        return;
    }

    if (m_imu.Set_FIFO_X_BDR(m_sample_rate) != LSM6DSO_OK)
    {
        Serial.println("Failed to set accelerometer FIFO batch rate!");
        return;
    }

    if (m_imu.Set_FIFO_G_BDR(m_sample_rate) != LSM6DSO_OK)
    {
        Serial.println("Failed to set gryoscope FIFO batch rate!");
        return;
    }
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

        m_imu.Get_FIFO_Tag(&tag); // TODO ERROR CHECK
        m_imu.Get_FIFO_Data(data); // TOOD ERROR CHECK

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
                Serial.println("Unknown tag");
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

        m_imu.Get_FIFO_Tag(&tag); // TODO ERROR CHECK

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                m_imu.Get_FIFO_X_Axes(data.data()); // TODO ERROR CHECK
                output[samples_read++].emplace<AccelData>(data);
                break;
            case LSM6DSO_GYRO_NC_TAG:
                m_imu.Get_FIFO_G_Axes(data.data()); // TODO ERROR CHECK
                output[samples_read++].emplace<GyroData>(data);
                break;
            default:
                Serial.println("Unknown tag");
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

    CircularQueue<std::pair<AccelData, uint32_t>, 32> accel_buffer;
    CircularQueue<std::pair<GyroData, uint32_t>, 32> gyro_buffer;

    uint32_t timestamp_us = 0;

    // Read all available FIFO samples
    while((data_samples + accel_buffer.size() + gyro_buffer.size()) < output.size() && samples_available)
    {
        uint8_t tag = 0;
        Eigen::Vector3<DataValueT> data;

        m_imu.Get_FIFO_Tag(&tag); // TODO ERROR CHECK

        switch(tag)
        {
            case LSM6DSO_XL_NC_TAG:
                m_imu.Get_FIFO_X_Axes(data.data()); // TODO ERROR CHECK
                accel_buffer.push({AccelData{data}, timestamp_us});
                break;
            case LSM6DSO_GYRO_NC_TAG:
                m_imu.Get_FIFO_G_Axes(data.data()); // TODO ERROR CHECK
                gyro_buffer.push({GyroData{data}, timestamp_us});
                break;
            default:
                Serial.println("Unknown tag");
                break;
        }

        samples_available--;
        timestamp_us += m_sample_time_us;

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
