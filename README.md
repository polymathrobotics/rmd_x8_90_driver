
# RMD X8 90 Driver library

The **RMD X8 90 Driver library** provides an interface for communicating with RMD motors via CAN bus. This library offers control commands, feedback processing, and error management with an integrated watchdog to monitor timeouts. Designed for high-precision motor control, it supports various command types and feedback processing.

## Features

- **Command Types**: Support for position, velocity, and current commands.
- **Feedback Types**: Receive motor feedback on position, velocity, configuration, and other states.
- **Error Management**: Detect and report errors like overheating, overcurrent, and low voltage.

## Classes

### `RmdX890Driver`

Main class providing motor control through CAN bus communication. Implements data packing and unpacking, watchdog functionality, and command/feedback management.

#### Constructor
```cpp
RmdX890Driver(
    const uint32_t & id,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(RMD_DEFAULT_TIMEOUT_MS),
    const bool & invert = false
);
```
- **Parameters**:
  - `id`: Unique motor ID.
  - `timeout`: Watchdog timeout period.
  - `invert`: Invert feedback for motor orientation adjustments.

#### Destructor
```cpp
~RmdX890Driver() = default;
```

### Member Functions

#### `unpackFeedback`
Unpacks CAN frame data into member variables.
```cpp
bool unpackFeedback(
    const std::array<unsigned char, CAN_MAX_DLC> & data,
    std::chrono::nanoseconds unpack_time
);
```

#### `packCommand`
Packs data into a CAN frame for sending commands.
```cpp
std::optional<polymath::socketcan::CanFrame> packCommand(
    const CommandType & command_type = CommandType::Velocity,
    const FeedbackType & feedback_type = FeedbackType::Velocity
);
```

#### Setters
Set motor command parameters.
- **`set_speed_cmd`**: Set speed command.
- **`set_current_cmd`**: Set current command.
- **`set_current_threshold`**: Set current threshold.

#### Getters
Retrieve feedback and status information.
- **`is_feedback_timed_out`**: Check if feedback timed out.
- **`get_motor_id`**: Get motor ID.
- **`get_speed_feedback`**: Get speed feedback.
- **`get_position_feedback`**: Get position feedback.
- **`get_current_feedback`**: Get current feedback.
- **`get_motor_temperature`**: Get motor temperature.
- **`get_error_feedback`**: Get error status.
- **`get_feedback_time`**: Get last feedback timestamp.

## Constants

- `RMD_DEFAULT_TIMEOUT_MS`: Default timeout (500 ms).

## Enums

- **`FeedbackType`**: Position and velocity, position only, etc.
- **`CommandType`**: Command types like position and velocity.
- **`ErrorFeedback`**: Error states such as overheating or overcurrent.

## Usage

1. Initialize `RmdX890Driver` with motor ID and timeout.
2. Use `set_speed_cmd` or `set_current_cmd` to set motor commands.
3. Use `packCommand` to create a CAN frame for commands.
4. Call `unpackFeedback` on received CAN frames to update motor status.
5. Check for errors using `get_error_feedback` and other getters.

## Dependencies

- [**socketcan_adapter**](git@github.com:polymathrobotics/socketcan_adapter.git): For CAN frame compatibility.

This driver is designed for robust motor control in robotics applications requiring precise and reliable CAN-based communication.
