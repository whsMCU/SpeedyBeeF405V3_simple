/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once


#include "flight/mixer.h"
#include "hw.h"


typedef struct mixerRuntime_s {
    uint8_t motorCount;
    motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

    bool feature3dEnabled;
    float motorOutputLow;
    float motorOutputHigh;
    float disarmMotorOutput;
    float deadbandMotor3dHigh;
    float deadbandMotor3dLow;

} mixerRuntime_t;

extern mixerRuntime_t mixerRuntime;

void mixerConfig_Init(void);
