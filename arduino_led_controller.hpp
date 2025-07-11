#ifndef ARDUINO_LED_CONTROLLER_HPP
#define ARDUINO_LED_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

using controller_interface::CallbackReturn;
using controller_interface::return_type;
namespace arduino_hardware
{

class ArduinoLEDController : public controller_interface::ControllerInterface
{
public:
    ArduinoLEDController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_;
};

}  // namespace arduino_blink

#endif  // ARDUINO_LED_CONTROLLER_HPP