#ifndef ARDUINO_HARWARE_HPP
#define ARDUINO_HARWARE_HPP

#include <memory>
#include <string>
#include <vector>
#include "serial/serial.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arduino_hardware{
class ArduinoLED : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoLED)

  // Initialize the hardware
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Configure the hardware
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Activate the hardware
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Deactivate the hardware
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Define state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Define command interfaces
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read from hardware (unused for LED)
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write to hardware
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const { return *logger_; }
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }
private:
  serial::Serial serr;
  double led_command_;
  double led_state_;
  std::string port_name_;
  int baud_rate_;
  int led_pin_;
  bool active_;
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
  

};

}  // namespace arduino_hardware

#endif  // ARDUINO_HARDWARE_HPP