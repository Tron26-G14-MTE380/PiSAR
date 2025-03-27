#include "pisar/mcp/driveunit/async_controller.h"

namespace pisar::mcp {

[[nodiscard]]
uint32_t AsyncDriveunitController::sendRequest(const driveunit_interface::Request& request)
{
    const uint32_t request_id = m_next_request_id.fetch_add(1, std::memory_order_relaxed);
    m_request_queue.push({request_id, request});
    return request_id;
}

[[nodiscard]]
std::optional<rd::expected<driveunit_interface::Response, std::error_code>>
AsyncDriveunitController::getResponse(uint32_t request_id)
{
    std::scoped_lock lock(m_response_mutex);
    auto it = m_response_map.find(request_id);

    if (it == m_response_map.end())
    {
        return std::nullopt; // Request hasn't been processed yet
    }

    auto result = std::move(it->second);
    m_response_map.erase(it);

    return result;
}


}