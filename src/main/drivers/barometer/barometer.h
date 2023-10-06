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

#include "hw.h"

struct baroDev_s;

#define BARO_SAMPLE_COUNT_MAX   48

typedef void (*baroOpFuncPtr)(struct baroDev_s *baro);                       // baro start operation
typedef bool (*baroGetFuncPtr)(struct baroDev_s *baro);                       // baro read/get operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature); // baro calculation (filled params are pressure and temperature)

// the 'u' in these variable names means 'uncompensated', 't' is temperature, 'p' pressure.
typedef struct baroDev_s {
    uint8_t ch;
    uint8_t address;
    bool combined_read;
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroGetFuncPtr read_ut;
    baroGetFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroGetFuncPtr read_up;
    baroGetFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baroDev_t;

typedef struct baro_s {
    baroDev_t dev;
    uint8_t baro_sample_count;              // size of baro filter array
    uint16_t baro_noise_lpf;                // additional LPF to reduce baro noise
    uint16_t baro_cf_vel;                   // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    uint16_t calibratingB;
    int32_t BaroAlt;
    int32_t baroTemperature;             // Use temperature for telemetry
    int32_t baroPressure;                // Use pressure for telemetry
} baro_t;
