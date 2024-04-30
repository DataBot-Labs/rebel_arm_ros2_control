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

#include <string>

namespace rebel_arm_hardware_interface
{
    class RebelArmJoint {
        
        public:
        std::string name = "";

        double command = 0.0;
        double position = 0.0;

        RebelArmJoint() = default;

        RebelArmJoint(const std::string &armJointName)
        {
            name = armJointName;
        }

        void updatePosition(int16_t position)
        {
            this->position = position;
        }
    };
}
