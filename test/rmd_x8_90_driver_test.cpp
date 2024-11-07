#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "rmd_x8_90_driver/rmd_x8_90_driver.hpp"
#include <iostream>
#include <iomanip>

using namespace polymath::drivers;

TEST_CASE("RmdX890Driver Constructor Test", "[RmdX890Driver]") {
  uint32_t id = 0x123;
  std::chrono::milliseconds timeout(500);

  RmdX890Driver driver(id, timeout);

  REQUIRE(driver.get_motor_id() == id);
  REQUIRE(driver.get_speed_feedback() == 0.0f);
  REQUIRE(driver.get_position_feedback() == 0.0f);
  REQUIRE(driver.get_current_feedback() == 0.0f);
  REQUIRE(driver.get_motor_temperature() == 0.0f);
  REQUIRE(driver.get_error_feedback() == ErrorFeedback::None);
}

TEST_CASE("RmdX890Driver Command Packing Test", "[RmdX890Driver]") {
  uint32_t id = 0x123;
  RmdX890Driver driver(id);
  std::array<unsigned char, 8> validation_array = {0x43, 0x42, 0xF6, 0xE9, 0x79, 0xFF, 0xFF, 0x00};

  driver.set_speed_cmd(123.456f);
  auto cmd_frame = driver.packCommand(CommandType::Velocity, FeedbackType::Velocity);
  REQUIRE(cmd_frame.has_value());
  auto frame = cmd_frame.value();

  auto result_data = frame.get_data();

  for (auto data : result_data) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(data);
  }

  std::cout << std::endl;

  REQUIRE(frame.get_id() == id);
  REQUIRE(frame.get_data() == validation_array);
}

TEST_CASE("RmdX890Driver Feedback Unpacking Test", "[RmdX890Driver]") {
  uint32_t id = 0x123;
  RmdX890Driver driver(id);

  std::array<unsigned char, CAN_MAX_DLC> data = {0x60, 0x42, 0xF6, 0xE9, 0x79, 0x15, 0x59, 0x66};
  // This correlates with:
  float expected_speed = 123.456f;
  float expected_current = 54.65f;

  float expected_motor_temperature = 26.0f;

  std::chrono::nanoseconds unpack_time(1000000);
  bool success = driver.unpackFeedback(data, unpack_time);
  REQUIRE(success);
  std::cout << "getting error" << std::endl;
  REQUIRE(driver.get_error_feedback() == ErrorFeedback::None);
  std::cout << "geting speed feedback" << std::endl;
  REQUIRE(driver.get_speed_feedback() == Approx(expected_speed));
  std::cout << "geting current" << std::endl;
  REQUIRE(driver.get_current_feedback() == Approx(expected_current));
  std::cout << "geting temperature" << std::endl;
  REQUIRE(driver.get_motor_temperature() == Approx(expected_motor_temperature));
  std::cout << "geting time" << std::endl;
  REQUIRE(driver.get_feedback_time() == unpack_time);
}
