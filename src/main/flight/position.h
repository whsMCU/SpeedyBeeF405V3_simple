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

#include "common/time.h"

#define POSITION_DEFAULT_ALT_NUM_SATS_GPS_USE 10
#define POSITION_DEFAULT_ALT_NUM_SATS_BARO_FALLBACK 7

typedef struct positionConfig_s {
    uint8_t altSource;
    uint8_t altNumSatsGpsUse;
    uint8_t altNumSatsBaroFallback;
} positionConfig_t;

extern positionConfig_t positionConfig;

void positionConfig_Init(void);

bool isAltitudeOffset(void);
void calculateEstimatedAltitude(uint32_t currentTimeUs);
int32_t getEstimatedAltitudeCm(void);
int16_t getEstimatedVario(void);
