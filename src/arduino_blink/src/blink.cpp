#include "blink.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>

namespace arduino_blink
{

CallbackReturn ArduinoLEDSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  port_name_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  led_command_ = 0.0;
  led_state_ = 0.0;
  active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoLEDSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoLEDSystem"), "Configuring hardware");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoLEDSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoLEDSystem"), "Activating hardware");
  
  try {
    serr.setPort(port_name_);
    serr.setBaudrate(baud_rate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serr.setTimeout(timeout);
    serr.open();
    if (!serr.isOpen()) {
      RCLCPP_FATAL(rclcpp::get_logger("ArduinoLEDSystem"), "Failed to open serial port: %s", port_name_.c_str());
      return CallbackReturn::FAILURE;
    }
    else{
      active_ = true;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinoLEDSystem"), "Connected to Arduino at %s", port_name_.c_str());
  } catch (const serial::IOException& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinoLEDSystem"), "IO Exception: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (const serial::SerialException& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinoLEDSystem"), "Serial Exception: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoLEDSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoLEDSystem"), "Deactivating hardware");
  try {
    if (serr.isOpen()) {
      serr.close();
    }
    active_ = false;
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoLEDSystem"), "Error closing port: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoLEDSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("led", "state", &led_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoLEDSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("led", "command", &led_command_);
  return command_interfaces;
}

return_type ArduinoLEDSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return return_type::OK;

  try {
    if (serr.available()) {
      std::string response = serr.readline();
      RCLCPP_DEBUG(rclcpp::get_logger("ArduinoLEDSystem"), "Received response: %s", response.c_str());
      try {
        // Convert to double (0.0 or 1.0)
        led_state_ = std::stod(response);
        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoLEDSystem"), "Read LED state: %f", led_state_);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoLEDSystem"), "Invalid state value: %s", response.c_str());
      }
    }
  } catch (const serial::PortNotOpenedException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoLEDSystem"), "Port not open: %s", e.what());
    return return_type::ERROR;
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoLEDSystem"), "Read failed: %s", e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type ArduinoLEDSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return return_type::OK;

  try {
    if (led_command_ == 1.0) {
      serr.write("1\n");
      RCLCPP_INFO(rclcpp::get_logger("ArduinoLEDSystem"), "Sent command to turn LED ON");
    } else if (led_command_ == 0.0) {
      serr.write("0\n");
      RCLCPP_INFO(rclcpp::get_logger("ArduinoLEDSystem"), "Sent command to turn LED OFF");
    }
  } catch (const serial::PortNotOpenedException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoLEDSystem"), "Port not open: %s", e.what());
    return return_type::ERROR;
  } catch (const serial::SerialException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoLEDSystem"), "Write failed: %s", e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

}  // namespace arduino_led_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arduino_blink::ArduinoLEDSystem, hardware_interface::SystemInterface)