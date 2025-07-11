#include "arduino_hardware/arduino_hardware.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"


namespace arduino_hardware
{

hardware_interface::CallbackReturn ArduinoLED::on_init(
  const hardware_interface::HardwareInfo &info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) != 
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.arduino_led"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
  
  port_name_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  led_pin_ = std::stoi(info_.hardware_parameters["led_pin"]);
  led_command_ = 0.0;
  led_state_ = 0.0;
  active_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoLED::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring hardware");
  try{
    RCLCPP_INFO(get_logger(), "Configuring serial port: %s at baud rate: %d", port_name_.c_str(), baud_rate_);
    serr.setPort(port_name_);
    RCLCPP_INFO(get_logger(), "Setting baud rate: %d", baud_rate_);
    serr.setBaudrate(baud_rate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serr.setTimeout(timeout);
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(get_logger(), "Error configuring serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoLED::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating hardware");

  try {
    serr.open();
    if (!serr.isOpen()) {
      RCLCPP_FATAL(
        get_logger(), "Failed to open serial port: %s", port_name_.c_str());
      return hardware_interface::CallbackReturn::FAILURE;
    }
    else{
      active_ = true;
    }
    RCLCPP_INFO(get_logger(), "Connected to Arduino at %s", port_name_.c_str());
  } catch (const serial::IOException& e) {
    RCLCPP_FATAL(get_logger(), "IO Exception: %s", e.what());
    return hardware_interface::CallbackReturn::FAILURE;
  } catch (const serial::SerialException& e) {
    RCLCPP_FATAL(get_logger(), "Serial Exception: %s", e.what());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoLED::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware");
  try {
    if (serr.isOpen()) {
      serr.close();
    }
    active_ = false;
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(get_logger(), "Error closing port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoLED::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("led", "state", &led_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoLED::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("led", "command", &led_command_);
  return command_interfaces;
}

hardware_interface::return_type ArduinoLED::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return hardware_interface::return_type::OK;

  try {
    if (serr.available()) {
      std::string response = serr.readline();
      RCLCPP_DEBUG(rclcpp::get_logger("ArduinoLED"), "Received response: %s", response.c_str());
      try {
        // Convert to double (0.0 or 1.0)
        led_state_ = std::stod(response);
        if(led_state_ == 1.0) {
          led_command_ = 0.0;  // Update command interface to match state
          RCLCPP_INFO(get_logger(), "LED is ON");
        } else if (led_state_ == 0.0) {
          led_command_ = 1.0;  // Update command interface to match state
          RCLCPP_INFO(get_logger(), "LED is OFF");
        } else {
          RCLCPP_WARN(get_logger(), "Received unexpected LED state: %f", led_state_);
        }
        RCLCPP_DEBUG(get_logger(), "Read LED state: %f", led_state_);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(get_logger(), "Invalid state value: %s", response.c_str());
      }
    }
  } catch (const serial::PortNotOpenedException& e) {
    RCLCPP_ERROR(get_logger(), "Port not open: %s", e.what());
    return hardware_interface::return_type::ERROR;
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(get_logger(), "Read failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoLED::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return hardware_interface::return_type::OK;

  try {
    if (led_command_ == 1.0) {
      serr.write("1\n");
      RCLCPP_INFO(get_logger(), "Sent command to turn LED ON");
    } else if (led_command_ == 0.0) {
      serr.write("0\n");
      RCLCPP_INFO(get_logger(), "Sent command to turn LED OFF");
    }
  } catch (const serial::PortNotOpenedException& e) {
    RCLCPP_ERROR(get_logger(), "Port not open: %s", e.what());
    return hardware_interface::return_type::ERROR;
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(get_logger(), "Write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arduino_led_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arduino_hardware::ArduinoLED, hardware_interface::SystemInterface
)