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

#include "common/sensor_alignment.h"

#include "hw.h"

#include "drivers/accgyro/accgyro.h"

#include "common/maths.h"

// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4,
    MAG_QMC5883 = 5,
    MAG_LIS3MDL = 6,
    MAG_MPU925X_AK8963 = 7
} magSensor_e;

struct magDev_s;

typedef bool (*sensorMagInitFuncPtr)(struct magDev_s *magdev);
typedef bool (*sensorMagReadFuncPtr)(struct magDev_s *magdev, int16_t *data);

typedef struct magDev_s {
    sensorMagInitFuncPtr init;                              // initialize function
    sensorMagReadFuncPtr read;                              // read 3 axis data function
    uint8_t ch;
    uint8_t address;
    sensor_align_e magAlignment;
    fp_rotationMatrix_t rotationMatrix;
    int16_t magGain[3];
} magDev_t;

typedef struct mag_s {
    uint8_t mag_alignment;                  // mag alignment
	magDev_t magDev;
	flightDynamicsTrims_t magZero;
    sensorAlignment_t mag_customAlignment;
    float magADC[XYZ_AXIS_COUNT];
} mag_t;
