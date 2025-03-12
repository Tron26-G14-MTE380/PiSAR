#include <array>
#include <iostream>
#include <vector>
#include <cassert>
#include "pisar/driveunit_interface/interface.h"

constexpr size_t kTestQueueSize = 4;

void runTest()
{
    using namespace pisar::driveunit_interface;

    RequestEncoder encoder;
    RequestDecoder<kTestQueueSize> decoder;

    // Define test packets
    HeartbeatRequest hb_request;
    CommandRequest rotate_cmd = CommandRotate{45.0f};

    Request request1 = hb_request;
    Request request2 = rotate_cmd;

    // Correct buffer sizing using kMaxEncodedPacketSize
    std::array<std::byte, RequestEncoder::kMaxEncodedPacketSize> buffer1;
    std::array<std::byte, RequestEncoder::kMaxEncodedPacketSize> buffer2;

    auto encoded1 = encoder.encode(request1, buffer1);
    auto encoded2 = encoder.encode(request2, buffer2);

    if (!encoded1 || !encoded2)
    {
        std::cerr << "Encoding failed!\n";
        return;
    }

    std::cout << "Encoded Packet 1 Size: " << encoded1->size() << "\n";
    std::cout << "Encoded Packet 2 Size: " << encoded2->size() << "\n";

    // Simulate SPI transmission (sending both packets in one go)
    std::vector<std::byte> spi_transmission;
    spi_transmission.insert(spi_transmission.end(), encoded1->begin(), encoded1->end());
    spi_transmission.insert(spi_transmission.end(), encoded2->begin(), encoded2->end());

    // Decode received data
    decoder.submit(std::span<const std::byte>(spi_transmission));

    size_t packetsDecoded = 0;
    while (decoder.packetsAvailable() > 0)
    {
        auto decoded_request = decoder.query();
        if (!decoded_request)
        {
            std::cerr << "Decoding failed!\n";
            return;
        }

        packetsDecoded++;

        if (std::holds_alternative<HeartbeatRequest>(*decoded_request))
        {
            std::cout << "Decoded Packet " << packetsDecoded << ": HeartbeatRequest\n";
        }
        else if (std::holds_alternative<CommandRequest>(*decoded_request))
        {
            auto cmd = std::get<CommandRequest>(*decoded_request);
            auto rotate_cmd = std::get<CommandRotate>(cmd);
            std::cout << "Decoded Packet " << packetsDecoded << ": CommandRotate with angle " << rotate_cmd.rotation_deg << "\n";
        }
        else
        {
            std::cerr << "Unknown packet type decoded!\n";
        }
    }

    std::cout << "Decoded " << packetsDecoded << " packets successfully!\n";

    // Check error reporting
    if (decoder.errorCount() > 0)
    {
        std::cerr << "Decoding errors detected: " << decoder.errorCount() << "\n";
    }
    else
    {
        std::cout << "No decoding errors detected.\n";
    }

    decoder.clearErrors();
}

int main()
{
    std::cout << "Running Packet Encoding/Decoding Test...\n";
    runTest();
    std::cout << "Test completed.\n";
    return 0;
}
