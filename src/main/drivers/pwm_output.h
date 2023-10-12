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

#include "common/time.h"

//#include "drivers/dma.h"
//#include "drivers/io_types.h"
#include "drivers/motor.h"
#include "hw/tim.h"

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 480

#define ALL_MOTORS 255

#define MOTOR_OUTPUT_LIMIT_PERCENT_MIN 1
#define MOTOR_OUTPUT_LIMIT_PERCENT_MAX 100

#define PWM_TIMER_1MHZ        MHZ_TO_HZ(1)

struct timerHardware_s;

typedef struct {
    volatile uint32_t 	*ccr;
    TIM_HandleTypeDef   *tim;
    uint8_t 			channel;
} timerChannel_t;

typedef enum {
    TIM_USE_ANY            = 0x0,
    TIM_USE_NONE           = 0x0,
    TIM_USE_PPM            = 0x1,
    TIM_USE_PWM            = 0x2,
    TIM_USE_MOTOR          = 0x4,
    TIM_USE_SERVO          = 0x8,
    TIM_USE_LED            = 0x10,
    TIM_USE_TRANSPONDER    = 0x20,
    TIM_USE_BEEPER         = 0x40,
    TIM_USE_CAMERA_CONTROL = 0x80,
} timerUsageFlag_e;

typedef struct {
    timerChannel_t channel;
    float pulseScale;
    float pulseOffset;
    bool forceOverflow;
    bool enabled;
    timerUsageFlag_e usageFlags;
    //IO_t io;
} pwmOutputPort_t;

extern pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

struct motorDevConfig_s;
motorDevice_t *motorPwmDevInit(const struct motorDevConfig_s *motorDevConfig, uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm);

pwmOutputPort_t *pwmGetMotors(void);
