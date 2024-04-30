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

#include <cstdint>

#define HEAD_FRAME 0xABCD

typedef struct {
    uint16_t head = HEAD_FRAME;
    int16_t joint1_position;
    int16_t joint2_position;
    int16_t joint3_position;
    int16_t joint4_position;
    int16_t joint5_position;
    int16_t joint6_position;
    uint16_t checksum;
} RebelArmControl;

typedef struct {
    uint16_t head = HEAD_FRAME;
    int16_t joint1_position;
    int16_t joint2_position;
    int16_t joint3_position;
    int16_t joint4_position;
    int16_t joint5_position;
    int16_t joint6_position;
    uint16_t checksum;
} RebelArmFeedback;
