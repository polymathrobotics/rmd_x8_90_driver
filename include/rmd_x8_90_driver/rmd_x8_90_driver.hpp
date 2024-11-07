#ifndef RMD_X8_90_DRIVER_HPP__RMD_X8_90_DRIVER_HPP
#define RMD_X8_90_DRIVER_HPP__RMD_X8_90_DRIVER_HPP

#include <cstdint>
#include <chrono>
#include <optional>
#include "socketcan_adapter/can_frame.hpp"

namespace polymath
{
namespace drivers
{

/// @brief Motor Feedback Types
enum class FeedbackType : uint8_t
{
  PositionAndVelocity = 1U,
  Position,
  Velocity,
  Configuration,
  Current
};

/// @brief Motor Command Types
enum class CommandType : uint8_t
{
  PositionHybrid = 0U,
  Position,
  Velocity,
  Current
};

/// @brief Error Enumeration
enum class ErrorFeedback : uint8_t
{
  None = 0U,
  Overheating,
  Overcurrent,
  LowVoltage,
  EncoderError,
  BrakeVoltageHigh,
  DrvError
};

// Default timeout if a motor message fails
constexpr uint32_t RMD_DEFAULT_TIMEOUT_MS = 500;

/// Can Message Transmission Packing Constants
/// TODO: Add additional command types
// Transmission Pack Command Type
constexpr uint64_t TX_COMMAND_TYPE_LENGTH = 3;
constexpr uint64_t TX_COMMAND_TYPE_OFFSET = 5;

// Transmission Feedback Type
constexpr uint64_t TX_FEEDBACK_TYPE_LENGTH = 2;
constexpr uint64_t TX_FEEDBACK_TYPE_OFFSET = 0;

// Transmission Speed Command
constexpr uint64_t TX_SPEED_COMMAND_LENGTH = 32;
constexpr uint64_t TX_SPEED_COMMAND_OFFSET = 8;

// Transmission Current Threshold
constexpr uint64_t TX_CURRENT_THRESHOLD_LENGTH = 16;
constexpr uint64_t TX_CURRENT_THRESHOLD_OFFSET = 40;

/// Can Message Reception Unpacking Constants
// Reception Message Feedback Type
constexpr uint64_t RX_FEEDBACK_TYPE_LENGTH = 3;
constexpr uint64_t RX_FEEDBACK_TYPE_OFFSET = 5;

// Reception Message Speed Feedback
constexpr uint64_t RX_ERROR_FEEDBACK_LENGTH = 5;
constexpr uint64_t RX_ERROR_FEEDBACK_OFFSET = 0;

// Reception Message Speed Feedback
constexpr uint64_t RX_SPEED_FEEDBACK_LENGTH = 32;
constexpr uint64_t RX_SPEED_FEEDBACK_OFFSET = 8;

// Reception Message Current Feedback
constexpr uint64_t RX_CURRENT_FEEDBACK_LENGTH = 16;
constexpr uint64_t RX_CURRENT_FEEDBACK_OFFSET = 40;

// Reception Message Motor Temperature
constexpr uint64_t RX_MOTOR_TEMPERATURE_LENGTH = 8;
constexpr uint64_t RX_MOTOR_TEMPERATURE_OFFSET = 56;

/// @brief Driver that processes RMD CAN data as well as implements a watchdog for timeouts
class RmdX890Driver
{
public:
  /// @brief Initialize CAN Driver for processing CAN Data
  /// @param id Identifier for the drive
  /// @param timeout Watchdog Timeout
  /// @param valid_window Watchdog Valid Window
  /// @param fault_timeout Watchdog how long to wait before faults timeout
  /// @param invert Whether to invert cmd and feedback due to motor orientation
  RmdX890Driver(
    const uint32_t & id,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(RMD_DEFAULT_TIMEOUT_MS),
    const bool & invert = false);

  /// @brief Default destructor
  ~RmdX890Driver() = default;

  /// @brief Unpack a can frame's data array into member variables
  /// @param data unsigned char, CAN_MAX_DLC sized data to parse
  /// @param unpack_time time received for unpacking
  /// @return success
  bool unpackFeedback(
    const std::array<unsigned char, CAN_MAX_DLC> & data,
    std::chrono::nanoseconds unpack_time);

  /// @brief Pack the data into a socketcan adapter compatible frame
  /// @param command_type Velocity defaulted, packing varies based on command
  /// @param feedback_type  Velocity defaulted, unpacking varies based on command
  /// @return Canframe if successful, otherwise std::nullopt
  std::optional<polymath::socketcan::CanFrame> packCommand(
    const CommandType & command_type = CommandType::Velocity,
    const FeedbackType & feedback_type = FeedbackType::Velocity);

  /// @brief Setter for speed as a float32
  /// @param speed_cmd
  void set_speed_cmd(const float & speed_cmd)
  {
    speed_cmd_ = speed_cmd;
  }

  /// @brief Setter for current as a float32
  /// @param current_cmd
  void set_current_cmd(const uint32_t & current_cmd)
  {
    current_cmd_ = current_cmd;
  }

  /// @brief Setter for current threshold as a uint16_t
  /// @param current_threshold
  void set_current_threshold(const uint16_t & current_threshold)
  {
    current_threshold_ = current_threshold;
  }

  /// @brief Get the state of the watchdog at the timestamp
  /// @param timestamp to check state at
  /// @return watchdog state
  bool is_feedback_timed_out(std::chrono::nanoseconds timestamp)
  {
    return (timestamp - feedback_time_) > timeout_;
  }

  /// @brief Get the saved motor id (cannot be changed once initialized)
  /// @return
  uint32_t get_motor_id()
  {
    return id_;
  }

  /// @brief Get the unpacked speed feedback IF EXISTS
  /// @return
  float get_speed_feedback()
  {
    return speed_feedback_;
  }

  /// @brief Get the unpacked position feedback IF EXISTS
  /// @return
  float get_position_feedback()
  {
    return position_feedback_;
  }

  /// @brief Get the unpacked current feedback IF EXISTS
  /// @return
  float get_current_feedback()
  {
    return current_feedback_;
  }

  /// @brief Get the unpacked motor temperature IF EXISTS
  /// @return
  float get_motor_temperature()
  {
    return motor_temperature_;
  }

  /// @brief Get the processed enumerated error IF EXISTS
  /// @return
  ErrorFeedback get_error_feedback()
  {
    return error_feedback_;
  }

  /// @brief Get time of feedback IF EXISTS
  /// @return
  std::chrono::nanoseconds get_feedback_time()
  {
    return feedback_time_;
  }

private:
  /// @brief Pack data reinterpreted into a uint64_t into the data buffer
  /// @param data_buffer buffer to pack data into (passed by reference, edited in place)
  /// @param data data to pack into the buffer (must be converted to uint64_t before packing)
  /// @param length number of BITS to pack
  /// @param offset offset in BITS on where to start packing
  /// @param big_endian whether we are big endian or not
  void packDataHelper(
    uint64_t & data_buffer, const uint64_t & data, const uint8_t & length,
    const uint8_t & offset, const bool & big_endian = false);

  /// @brief Unpack data into a uint64_t that must be reinterpreted if necessary
  /// @param data_buffer buffer to unpack data from
  /// @param length number of BITS to unpack
  /// @param offset offset in BITS on where to start unpacking
  /// @param big_endian whether we are big endian or not
  /// @return unpacked uint64_t
  uint64_t unpackDataHelper(
    const uint64_t & data_buffer, const uint8_t & length,
    const uint8_t & offset, const bool & big_endian = false);

  /// @brief Swap the bytes (used for endianness)
  /// @param data data as a char array/pointer that needs to be swapped
  /// @param length of the data that needs to be swapped
  //  * CAN GET SEGMENTATION FAULTS IF YOU GO PAST THE LENGTH OF THE ORIGINAL DATA TYPE
  void byteswap(unsigned char * data, size_t length);

  uint32_t id_;
  float speed_cmd_;
  uint32_t current_cmd_;
  uint16_t current_threshold_;

  // All outputs get converted into floats
  float speed_feedback_;
  float position_feedback_;
  float current_feedback_;
  float motor_temperature_;

  // Inversion factor is either +1 or -1 and multiplies feedback and cmd
  float inversion_factor_;

  // Error feedback information
  ErrorFeedback error_feedback_;

  std::chrono::nanoseconds feedback_time_;
  std::chrono::nanoseconds timeout_;
};

} // namespace drivers
} // namespace polymath

#endif // RMD_X8_90_DRIVER_HPP__RMD_X8_90_DRIVER_HPP
