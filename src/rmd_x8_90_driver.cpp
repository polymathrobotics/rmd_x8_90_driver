#include "rmd_x8_90_driver/rmd_x8_90_driver.hpp"
#include <climits>
#include <iostream>

namespace polymath
{
namespace drivers
{
RmdX890Driver::RmdX890Driver(
  const uint32_t & id,
  const std::chrono::milliseconds & timeout,
  const bool & invert)
: id_(id),
  speed_cmd_(0.0),
  current_cmd_(0.0),
  current_threshold_(0xFFFF),
  speed_feedback_(0.0),
  position_feedback_(0.0),
  current_feedback_(0.0),
  motor_temperature_(0.0),
  inversion_factor_(1.0),
  error_feedback_(ErrorFeedback::None),
  feedback_time_(std::chrono::nanoseconds(0))
{
  if (invert) {
    inversion_factor_ = -1;
  }

  timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
}

std::optional<socketcan::CanFrame> RmdX890Driver::packCommand(
  const CommandType & command_type,
  const FeedbackType & feedback_type)
{
  if (command_type == CommandType::Velocity) {
    polymath::socketcan::CanFrame frame;
    frame.set_can_id(static_cast<canid_t>(id_));
    frame.set_id_as_standard();
    frame.set_type_data();
    frame.set_len(7);

    auto data_buffer = std::array<unsigned char, CAN_MAX_DLC>{0};
    /// TODO: No magic numbers
    /// TODO: Add pack functionality directly into the CAN message so we don't need to copy
    // Pack command type
    uint64_t & buffer_bits = *reinterpret_cast<uint64_t *>(data_buffer.data());

    // Pack Command
    const uint8_t command_type_uint = static_cast<uint8_t>(command_type);
    packDataHelper(buffer_bits, command_type_uint, TX_COMMAND_TYPE_LENGTH, TX_COMMAND_TYPE_OFFSET);

    // Pack Feedback
    const uint8_t feedback_type_uint = static_cast<uint8_t>(feedback_type);
    packDataHelper(
      buffer_bits, feedback_type_uint, TX_FEEDBACK_TYPE_LENGTH,
      TX_FEEDBACK_TYPE_OFFSET);

    // Pack speed
    float active_speed_command = speed_cmd_ * inversion_factor_;
    uint64_t speed_bits = *reinterpret_cast<uint32_t *>(&active_speed_command);
    packDataHelper(buffer_bits, speed_bits, TX_SPEED_COMMAND_LENGTH, TX_SPEED_COMMAND_OFFSET, true);

    // Pack current threshold
    packDataHelper(
      buffer_bits, current_threshold_, TX_CURRENT_THRESHOLD_LENGTH,
      TX_CURRENT_THRESHOLD_OFFSET, true);

    frame.set_data(data_buffer);
    return std::optional<socketcan::CanFrame>(frame);
  } else {
    /// TODO: Add implementations for the other command types
    return std::nullopt;
  }
}

bool RmdX890Driver::unpackFeedback(
  const std::array<unsigned char, CAN_MAX_DLC> & data,
  std::chrono::nanoseconds unpack_time)
{
  /// TODO: Make life easier and add a method that returns a value instead of edits a reference
  const uint64_t & buffer_bits = *reinterpret_cast<const uint64_t *>(data.data());

  auto feedback = unpackDataHelper(buffer_bits, RX_FEEDBACK_TYPE_LENGTH, RX_FEEDBACK_TYPE_OFFSET);

  uint64_t placeholder;

  if (feedback == static_cast<uint8_t>(FeedbackType::Velocity)) {
    feedback_time_ = unpack_time;

    // // Unpack  error
    uint8_t error_message =
      static_cast<uint8_t>(unpackDataHelper(
        buffer_bits, RX_ERROR_FEEDBACK_LENGTH,
        RX_ERROR_FEEDBACK_OFFSET));
    error_feedback_ = ErrorFeedback(error_message);

    // Unpack Speed
    placeholder = unpackDataHelper(
      buffer_bits, RX_SPEED_FEEDBACK_LENGTH, RX_SPEED_FEEDBACK_OFFSET,
      true);
    float speed = *reinterpret_cast<float *>(&placeholder);
    speed_feedback_ = inversion_factor_ * speed;

    // // Unpack motor phase current
    // // TODO: Add constant for scaling current
    placeholder = unpackDataHelper(
      buffer_bits, RX_CURRENT_FEEDBACK_LENGTH,
      RX_CURRENT_FEEDBACK_OFFSET, true);
    int16_t current_feedback_int = *reinterpret_cast<int16_t *>(&placeholder);
    current_feedback_ = inversion_factor_ * static_cast<float>(current_feedback_int) * 0.01;

    // /// TODO: Move scaling into some other functionality and create consants for scaling
    placeholder = unpackDataHelper(
      buffer_bits, RX_MOTOR_TEMPERATURE_LENGTH,
      RX_MOTOR_TEMPERATURE_OFFSET, true);
    float raw_motor_temperature = static_cast<float>(placeholder);
    motor_temperature_ = (raw_motor_temperature - 50) / 2;

    return true;
  } else {
    /// TODO: Add implementations for the other feedback types
    return false;
  }
}

void RmdX890Driver::packDataHelper(
  uint64_t & data_buffer, const uint64_t & data,
  const uint8_t & length, const uint8_t & offset,
  const bool & big_endian)
{
  uint64_t mask = ((1ULL << length) - 1) << offset;

  uint64_t editable_data = data;
  if (big_endian) {
    byteswap(reinterpret_cast<unsigned char *>(&editable_data), length / CHAR_BIT);
  }

  // clear the part of the data message that the mask covers
  data_buffer &= ~mask;
  data_buffer |= (editable_data << offset) & mask;
}

uint64_t RmdX890Driver::unpackDataHelper(
  const uint64_t & data_buffer, const uint8_t & length,
  const uint8_t & offset, const bool & big_endian)
{
  uint64_t mask = ((1ULL << length) - 1);

  uint64_t result = 0;
  result = (data_buffer >> offset) & mask;

  if (big_endian) {
    byteswap(reinterpret_cast<unsigned char *>(&result), length / CHAR_BIT);
  }

  return result;
}

void RmdX890Driver::byteswap(unsigned char * data, size_t length)
{
  for (size_t i = 0; i < length / 2; ++i) {
    std::swap(data[i], data[length - 1 - i]);
  }
}


} // drivers
} // polymath
