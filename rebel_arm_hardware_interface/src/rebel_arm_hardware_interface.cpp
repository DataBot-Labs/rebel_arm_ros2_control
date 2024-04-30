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

        hardwareConfig.position1JointName = info.hardware_parameters.at("position1_joint_name");
        hardwareConfig.position2JointName = info.hardware_parameters.at("position2_joint_name");
        hardwareConfig.position3JointName = info.hardware_parameters.at("position3_joint_name");
        hardwareConfig.position4JointName = info.hardware_parameters.at("position4_joint_name");
        hardwareConfig.position5JointName = info.hardware_parameters.at("position5_joint_name");
        hardwareConfig.position6JointName = info.hardware_parameters.at("position6_joint_name");

        serialPortConfig.device = info.hardware_parameters.at("device");
        serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
        serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

        for (const hardware_interface::ComponentInfo & joint : info.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(), joint.state_interfaces.size());
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                
                return hardware_interface::CallbackReturn::ERROR;
            }
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
