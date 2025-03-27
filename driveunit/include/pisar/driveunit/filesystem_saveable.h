#pragma once

// Needed for serialization support for some things like Eigen
#include "pisar/driveunit_interface/zpp_support.h"

#include <FS.h>
#include <LittleFS.h>
#include <zpp_bits.h>

#include <string_view>

namespace pisar::driveunit {

template<class TDerived>
class FileSystemSaveable
{
public:
    /**
     * @brief Saves data to LittleFS.
     * @param file_path The location to save to.
     * @return True if the data was successfully saved, false otherwise.
     */
    [[nodiscard]] bool save(std::string_view file_path) const
    {
        // Serialize the calibration data
        auto [data_buffer, encoder] = zpp::bits::data_out();
    
        if (zpp::bits::failure(encoder(getData())))
        {
            PISAR_LOG_ERROR("Failed to encode data!");
            return false;
        }
    
        // Write to LittleFS
        File file = LittleFS.open(file_path.data(), "w");
        if (!file)
        {
            PISAR_LOG_ERROR("Failed to open file for writing!");
            return false;
        }
    
        auto serialized_data = encoder.processed_data();
        const size_t bytes_written = file.write(reinterpret_cast<const uint8_t*>(serialized_data.data()), serialized_data.size());
        file.close();
    
        if (bytes_written != serialized_data.size())
        {
            PISAR_LOG_ERROR("Failed to write data to file! Only %u bytes written.", bytes_written);
            return false;
        }
    
        return true;
    }

    /**
     * @brief Loads data from LittleFS.
     * @param file_path The location to load from.
     * @return True if the data was successfully loaded, false otherwise.
     */
    [[nodiscard]] bool load(std::string_view file_path)
    {
        // Open file
        File file = LittleFS.open(file_path.data(), "r");
        if (!file)
        {
            PISAR_LOG_ERROR("Calibration data file not found. Calibration loading failed!");
            return false;
        }
    
        // Read data into buffer
        std::vector<std::byte> data_buffer(file.size());
        const int bytes_read = file.read(reinterpret_cast<uint8_t*>(data_buffer.data()), data_buffer.size());
        file.close();
    
        if (bytes_read == 0)
        {
            PISAR_LOG_ERROR("Failed to read calibration data from file!");
            return false;
        }
    
        // Deserialize calibration data
        auto decoder = zpp::bits::in(std::span(data_buffer.data(), bytes_read));
    
        if (zpp::bits::failure(decoder(getData())))
        {
            PISAR_LOG_ERROR("Failed to decode calibration data from file!");
            return false;
        }
    
        return true;
    }
    
private:
    [[nodiscard]] TDerived& getData()
    {
        return *reinterpret_cast<TDerived*>(this);
    }

    [[nodiscard]] const TDerived& getData() const
    {
        return *reinterpret_cast<const TDerived*>(this);
    }
};

}