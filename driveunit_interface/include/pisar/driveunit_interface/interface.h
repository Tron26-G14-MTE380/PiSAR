#pragma once

#include "pisar/driveunit_interface/message.h"
#include "pisar/driveunit_interface/codec.h"

#include <Eigen/Dense>
#include <zpp_bits/zpp_bits.h>

#include <chrono>

namespace std::chrono {
/**
 * @brief Serialization support for std::chrono::duration<T, Ratio>
 */
template <typename TRep, typename TRatio>
constexpr auto serialize(auto &archive, const std::chrono::duration<TRep, TRatio> &duration)
{
    return archive(duration.count());
}
}

namespace pisar::driveunit_interface
{
namespace detail {
template <typename Matrix> class EigenMatrix : public std::false_type {};
template <typename Scalar, int Rows, int Cols, int Options>
  class EigenMatrix<Eigen::Matrix<Scalar, Rows, Cols, Options>> : public std::true_type {};
}
template <typename Matrix> concept CIsEigenMatrix = detail::EigenMatrix<Matrix>::value;
}

namespace Eigen {
/**
 * @brief Serialization support for Eigen::Matrix<T, Rows, Cols>.
 *
 * Taken from https://github.com/eyalz800/zpp_bits/issues/149.
 */
template <typename Matrix>
constexpr auto serialize(auto & archive, Matrix& matrix) requires pisar::driveunit_interface::CIsEigenMatrix<std::remove_cvref_t<Matrix>>
{
  // this function will be called twice, first to get how many members to archive, then to archive the members
  // first call
  if constexpr (std::integral<decltype(archive())>)
    return 1 + (Matrix::CompileTimeTraits::RowsAtCompileTime == Eigen::Dynamic)
      + (Matrix::CompileTimeTraits::ColsAtCompileTime == Eigen::Dynamic);
  // second call
  else
  {
    typename Matrix::Index nRow, nCol;
    std::vector<typename Matrix::Scalar> data;
    if constexpr (archive.kind() == zpp::bits::kind::out)
      { nRow = matrix.rows(); nCol = matrix.cols(); data = std::vector(matrix.data(), matrix.data() + matrix.size()); }
    zpp::bits::errc result;
    if constexpr (Matrix::CompileTimeTraits::RowsAtCompileTime == Eigen::Dynamic)
      { if (result = archive(nRow); result.code != std::errc{}) [[unlikely]] return result; }
    else nRow = Matrix::CompileTimeTraits::RowsAtCompileTime;
    if constexpr (Matrix::CompileTimeTraits::ColsAtCompileTime == Eigen::Dynamic)
      { if (result = archive(nCol); result.code != std::errc{}) [[unlikely]] return result; }
    else nCol = Matrix::CompileTimeTraits::ColsAtCompileTime;
    result = archive(data);
    if (result.code != std::errc{}) [[unlikely]] return result;
    if constexpr (archive.kind() == zpp::bits::kind::in)
      matrix = Eigen::Map<const Matrix>(data.data(), nRow, nCol);
    return result;
  }
}
}

namespace pisar::driveunit_interface {

constexpr size_t kUartSpeed = 1'500'000;;
constexpr size_t kMaxRequestPacketSize = 256;
constexpr size_t kMaxResponsePacketSize = 64;

struct DefaultResponse
{
    using serialize = zpp::bits::members<1>;

    bool ack;
};

// Heartbeat
struct HeartbeatRequest
{
    using serialize = zpp::bits::members<0>;
};

struct HeartbeatResponse
{
    using serialize = zpp::bits::members<1>;

    std::chrono::duration<uint32_t, std::milli> time_alive;
};

// Commands

struct CommandIdle
{
    using serialize = zpp::bits::members<0>;
};

struct CommandFollowTrajectory
{
    using serialize = zpp::bits::members<2>;
    using ReferenceTimeT = std::chrono::microseconds;

    ReferenceTimeT reference_time;
    std::vector<Eigen::Vector2f> trajectory;
};

struct CommandRotate
{
    using serialize = zpp::bits::members<1>;

    float rotation_deg; // CCW positive
};

using CommandRequest = MessageSet<CommandIdle, CommandFollowTrajectory, CommandRotate>;
using CommandResponse = DefaultResponse;


using Request = MessageSet<HeartbeatRequest, CommandRequest>;
using Response = MessageSet<HeartbeatResponse, CommandResponse>;

using RequestEncoder = PacketEncoder<Request, kMaxRequestPacketSize>;

template<size_t tkPacketQueueSize>
using RequestDecoder = PacketDecoder<Request, kMaxRequestPacketSize, tkPacketQueueSize>;


using ResponseEncoder = PacketEncoder<Response, kMaxResponsePacketSize>;
template<size_t tkPacketQueueSize>
using ResponseDecoder = PacketDecoder<Response, kMaxResponsePacketSize, tkPacketQueueSize>;


};