#pragma once

#ifdef __linux__
#include <wiringPi.h>
#include <wiringSerial.h>
#endif

#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <optional>
#include <chrono>
#include <iostream>

namespace pisar::mcp {

#ifdef __linux__

/**
 * @brief UART serial communication wrapper for Raspberry Pi using WiringPi.
 *
 * Provides buffered reading with optional timeouts, non-blocking writes, and
 * efficient handling of UART data via file descriptors.
 */
class SerialUart
{
private:
    int m_fd;                     ///< File descriptor for the serial port.
    std::string m_port;           ///< Path to the serial device (e.g., "/dev/serial0").

public:
    /**
     * @brief Constructs the SerialUart object.
     * @param port The device file (e.g., "/dev/serial0").
     */
    explicit SerialUart(const std::string_view port) : m_fd(-1), m_port(port) {}

    /**
     * @brief Destructor to ensure the serial port is closed.
     */
    inline ~SerialUart()
    {
        close();
    }

    /// @brief Returns the uart device name.
    [[nodiscard]] inline std::string_view device() { return m_port; }

    /**
     * @brief Opens the serial port using WiringPi.
     * @param baud_rate Baud rate (e.g., 115200).
     * @return True if successful, false otherwise.
     */
    [[nodiscard]] inline bool open(int baud_rate)
    {
        wiringPiSetup(); // Initializes wiringPi using wiringPi's simlified number system.
        m_fd = serialOpen(m_port.c_str(), baud_rate);
        if (m_fd < 0)
        {
            return false;
        }

        serialFlush(m_fd); // Clear any existing data in the buffers
        return true;
    }

    /**
     * @brief Closes the serial port.
     */
    inline void close()
    {
        if (isOpen())
        {
            ::close(m_fd);
            m_fd = -1;
        }
    }

    /**
     * @brief Checks if the serial port is open.
     * @return True if open, false otherwise.
     */
    [[nodiscard]] inline bool isOpen() const { return m_fd >= 0; }

    /**
     * @brief Writes data to the serial port.
     * @param buffer The byte buffer to send.
     * @return The number of bytes successfully written.
     */
    [[nodiscard]] inline size_t write(const std::span<const std::byte>& buffer) const
    {
        if (!isOpen())
        {
            return 0;
        }

        return ::write(m_fd, reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());
    }

    /**
     * @brief Writes data to the serial port.
     * @param data The byte to send.
     * @return True if successful otherwise false.
     */
    [[nodiscard]] inline bool write(const std::byte data) const
    {
        return write(std::span(&data, 1)) == 1;
    }

    /**
     * @brief Return the number of bytes of data avalable to be read in the serial port
     *
     * @return The number of bytes available for reading or std::nullopt on failure.
     */
    [[nodiscard]] inline std::optional<size_t> available() const
    {
        if (!isOpen())
        {
            return std::nullopt;
        }

        const int result = serialDataAvail(m_fd);
        if (result == -1)
        {
            return std::nullopt;
        }

        return result;
    }

    /**
     * @brief Reads data from the serial port (blocking or non-blocking).
     * @param buffer The buffer to read into..
     * @param timeout Optional timeout (0 = non-blocking).
     * @return The number of
     */
    inline std::optional<size_t> read(
        const std::span<std::byte> buffer, const std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) const
    {
        if (!isOpen())
        {
            return std::nullopt;
        }

        fd_set read_fds;
        struct timeval tv;

        FD_ZERO(&read_fds);
        FD_SET(m_fd, &read_fds);

        tv.tv_sec = timeout.count() / 1000;
        tv.tv_usec = (timeout.count() % 1000) * 1000;

        const int result = select(m_fd + 1, &read_fds, nullptr, nullptr, timeout.count() > 0 ? &tv : nullptr);

        if (result <= 0)
        {
            return std::nullopt;
        }

        const size_t bytes_read = ::read(m_fd, buffer.data(), buffer.size());
        if (bytes_read == 0)
        {
            return std::nullopt;
        }

        return bytes_read;
    }

    /**
     * @brief Reads a single byte from the serial port.
     * @param timeout Timeout duration (0 for non-blocking).
     * @return std::optional<uint8_t> The received byte, or std::nullopt if nothing received.
     */
    inline std::optional<std::byte> readByte(const std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) const
    {
        std::byte data = std::byte(0);
        const auto result = read(std::span(&data, 1), timeout);
        if (!result || result.value() != 1)
        {
            return std::nullopt;
        }
        return data;
    }

    /**
     * @brief Flushes input and output buffers.
     */
    inline void flush() const
    {
        if (isOpen())
        {
            serialFlush(m_fd);
        }
    }
};

#else

/**
 * @brief UART serial communication wrapper for Raspberry Pi using WiringPi.
 *
 * Provides buffered reading with optional timeouts, non-blocking writes, and
 * efficient handling of UART data via file descriptors.
 */
class SerialUart
{
private:
    int m_fd;                     ///< File descriptor for the serial port.
    std::string m_port;           ///< Path to the serial device (e.g., "/dev/serial0").

public:
    /**
     * @brief Constructs the SerialUart object.
     * @param port The device file (e.g., "/dev/serial0").
     */
    explicit SerialUart(const std::string_view port) : m_fd(-1), m_port(port) {}

    /// @brief Returns the uart device name.
    [[nodiscard]] inline std::string_view device() { return m_port; }

    /**
     * @brief Opens the serial port using WiringPi.
     * @param baud_rate Baud rate (e.g., 115200).
     * @return True if successful, false otherwise.
     */
    [[nodiscard]] inline bool open(int baud_rate) { return false; }

    /**
     * @brief Closes the serial port.
     */
    inline void close() {}

    /**
     * @brief Checks if the serial port is open.
     * @return True if open, false otherwise.
     */
    [[nodiscard]] inline bool isOpen() const { return false; }

    /**
     * @brief Writes data to the serial port.
     * @param buffer The byte buffer to send.
     * @return The number of bytes successfully written.
     */
    [[nodiscard]] inline size_t write(const std::span<const std::byte>& buffer) const { return 0; }

    /**
     * @brief Writes data to the serial port.
     * @param data The byte to send.
     * @return True if successful otherwise false.
     */
    [[nodiscard]] inline bool write(const std::byte data) const { return false; }

    /**
     * @brief Return the number of bytes of data avalable to be read in the serial port
     *
     * @return The number of bytes available for reading or std::nullopt on failure.
     */
    [[nodiscard]] inline std::optional<size_t> available() const { return 0; }

    /**
     * @brief Reads data from the serial port (blocking or non-blocking).
     * @param buffer The buffer to read into..
     * @param timeout Optional timeout (0 = non-blocking).
     * @return The number of
     */
    inline std::optional<size_t> read(
        const std::span<std::byte> buffer, const std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) const
    {
        return std::nullopt;
    }

    /**
     * @brief Reads a single byte from the serial port.
     * @param timeout Timeout duration (0 for non-blocking).
     * @return std::optional<uint8_t> The received byte, or std::nullopt if nothing received.
     */
    inline std::optional<std::byte> readByte(const std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) const
    {
        return std::nullopt;
    }

    /**
     * @brief Flushes input and output buffers.
     */
    inline void flush() const {}
};

#endif

}