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

#pragma once

#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rebel_arm_hardware_interface
{
    class RebelArmHardwareInterface : public hardware_interface::SystemInterface
    {
        struct SerialPortConfig
        {
            std::string device = "/dev/ttyUSB0";
            int baudRate = 9600;
            int timeout = 1000;
        };

        public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RebelArmHardwareInterface)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

        // void feedbackCallback(MotorWheelFeedback);
    };
}