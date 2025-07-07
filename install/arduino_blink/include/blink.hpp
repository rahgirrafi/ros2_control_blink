#ifndef BLINK_HPP
#define BLINK_HPP

#include <string>
#include <vector>
#include <serial/serial.h>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace arduino_blink{
class ArduinoLEDSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoLEDSystem)

  // Initialize the hardware
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Configure the hardware
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Activate the hardware
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Deactivate the hardware
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Define state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Define command interfaces
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read from hardware (unused for LED)
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write to hardware
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  serial::Serial serr;
  double led_command_;
  double led_state_;
  std::string port_name_;
  int baud_rate_;
  bool active_;
};

}  // namespace arduino_led_hardware

#endif  // ARDUINO_LED_HARDWARE__ARDUINO_LED_SYSTEM_HPP_