// Copyright 2024 Robert Gruberski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rebel_arm_hardware_interface/rebel_arm_hardware_interface.hpp"

namespace rebel_arm_hardware_interface
{
    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RebelArmHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RebelArmHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RebelArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) // period
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RebelArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(rebel_arm_hardware_interface::RebelArmHardwareInterface, hardware_interface::SystemInterface)
