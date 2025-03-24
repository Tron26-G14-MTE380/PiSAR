#include "pisar/mcp/driveunit/transport.h"

namespace pisar::mcp {

[[nodiscard]]
rd::expected<driveunit_interface::Response, std::error_code>
DriveunitTransport::sendRequestInternal(const driveunit_interface::Request& request)
{
    if (m_uart.isOpen() == false)
    {
        return rd::unexpected(make_error_code(Error::kUartNotOpen));
    }

    std::scoped_lock lock(m_mutex);

    // Encode the request directly into a buffer
    std::array<std::byte, driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> send_buffer{};

    driveunit_interface::RequestEncoder encoder;
    auto encoded = encoder.encode(request, std::span(send_buffer));

    if (!encoded)
    {
        return rd::unexpected(make_error_code(Error::kEncodingError));
    }

    // Write request to UART
    if (m_uart.write(encoded.value()) != encoded->size())
    {
        return rd::unexpected(make_error_code(Error::kWriteError));
    }

    // Read response from UART
    return receiveResponse();
}

[[nodiscard]]
rd::expected<driveunit_interface::Response, std::error_code>
DriveunitTransport::receiveResponse(std::chrono::milliseconds timeout)
{
    driveunit_interface::ResponseDecoder<1> decoder;
    std::array<std::byte, driveunit_interface::ResponseEncoder::kMaxEncodedPacketSize> read_buffer;

    const auto start_time = std::chrono::steady_clock::now();

    while (true)
    {
        // Check if timeout has been exceeded
        const auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        if (elapsed_time >= timeout)
        {
            return rd::unexpected(make_error_code(Error::kReadTimeout));
        }

        // Calculate remaining time for this read operation
        const auto remaining_time = std::chrono::duration_cast<std::chrono::milliseconds>(timeout - elapsed_time);

        const auto bytes_read = m_uart.read(read_buffer, remaining_time);
        if (!bytes_read)
        {
            return rd::unexpected(make_error_code(Error::kReadTimeout)); // Properly fail if no bytes were read
        }

        // Submit received data to the decoder
        decoder.submit(std::span(read_buffer.data(), bytes_read.value()));

        // Try to extract a packet
        if (decoder.packetsAvailable())
        {
            const auto response = decoder.query();
            if (response)
            {
                return response.value();
            }
            else
            {
                return rd::unexpected(make_error_code(Error::kDecodingError));
            }
        }
    }
}

}