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

#include "serial_port_service.hpp"
#include "rebel_arm_joint.hpp"

namespace rebel_arm_hardware_interface
{
    class RebelArmHardwareInterface : public hardware_interface::SystemInterface
    {
        struct HardwareConfig
        {
            std::string position1JointName = "position1_joint";
            std::string position2JointName = "position2_joint";
            std::string position3JointName = "position3_joint";
            std::string position4JointName = "position4_joint";
            std::string position5JointName = "position5_joint";
            std::string position6JointName = "position6_joint";
        };

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

        void rebelArmFeedbackCallback(RebelArmFeedback);

        private:

        SerialPortService serialPortService;

        HardwareConfig hardwareConfig;
        SerialPortConfig serialPortConfig;

        RebelArmJoint rebel_arm_joint1;
        RebelArmJoint rebel_arm_joint2;
        RebelArmJoint rebel_arm_joint3;
        RebelArmJoint rebel_arm_joint4;
        RebelArmJoint rebel_arm_joint5;
        RebelArmJoint rebel_arm_joint6;
    };
}