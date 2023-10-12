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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hw.h"

#ifdef USE_MOTOR

#include "common/maths.h"

#include "drivers/pwm_output.h" // for PWM_TYPE_* and others

#include "rx/rc_controls.h" // for flight3DConfig_t

#include "drivers/motor.h"

static motorDevice_t *motorDevice;

motorConfig_t motorConfig;

void motorConfig_Init(void)
{

	motorConfig.minthrottle = 1070;
	motorConfig.dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
	motorConfig.dev.motorPwmProtocol = PWM_TYPE_MULTISHOT;

    motorConfig.maxthrottle = 2000;
    motorConfig.mincommand = 1000;
    motorConfig.digitalIdleOffsetValue = 550;

    motorConfig.motorPoleCount = 14;   // Most brushes motors that we use are 14 poles

    motorConfig.motor_output_limit = 100;

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorConfig.dev.motorOutputReordering[i] = i;
    }
}

static bool motorProtocolEnabled = false;
static bool motorProtocolDshot = false;

void motorShutdown(void)
{
    motorDevice->vTable.shutdown();
    motorDevice->enabled = false;
    motorDevice->motorEnableTimeMs = 0;
    motorDevice->initialized = false;
    delayMicroseconds(1500);
}

void motorWriteAll(float *values)
{
#ifdef USE_PWM_OUTPUT
    if (motorDevice->enabled) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (!motorDevice->vTable.updateStart()) {
            return;
        }
#endif
        for (int i = 0; i < motorDevice->count; i++) {
            motorDevice->vTable.write(i, values[i]);
        }
        motorDevice->vTable.updateComplete();
    }
#endif
}

unsigned motorDeviceCount(void)
{
    return motorDevice->count;
}

motorVTable_t motorGetVTable(void)
{
    return motorDevice->vTable;
}

// This is not motor generic anymore; should be moved to analog pwm module
static void analogInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow) {
	*disarm = motorConfig->mincommand;
	*outputLow = motorConfig->minthrottle;
	*outputHigh = motorConfig->maxthrottle - ((motorConfig->maxthrottle - motorConfig->minthrottle) * (1 - outputLimit));
}

bool checkMotorProtocolEnabled(const motorDevConfig_t *motorDevConfig, bool *isProtocolDshot)
{
    bool enabled = false;
    bool isDshot = false;

    switch (motorDevConfig->motorPwmProtocol) {
    case PWM_TYPE_STANDARD:
    case PWM_TYPE_ONESHOT125:
    case PWM_TYPE_ONESHOT42:
    case PWM_TYPE_MULTISHOT:
    case PWM_TYPE_BRUSHED:
        enabled = true;

        break;

#ifdef USE_DSHOT
    case PWM_TYPE_DSHOT150:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_PROSHOT1000:
        enabled = true;
        isDshot = true;

        break;
#endif
    default:

        break;
    }

    if (isProtocolDshot) {
        *isProtocolDshot = isDshot;
    }

    return enabled;
}

static void checkMotorProtocol(const motorDevConfig_t *motorDevConfig)
{
    motorProtocolEnabled = checkMotorProtocolEnabled(motorDevConfig, &motorProtocolDshot);
}

// End point initialization is called from mixerInit before motorDevInit; can't use vtable...
void motorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    checkMotorProtocol(&motorConfig->dev);

    if (isMotorProtocolEnabled()) {
        if (!isMotorProtocolDshot()) {
            analogInitEndpoints(motorConfig, outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
        }
#ifdef USE_DSHOT
        else {
            dshotInitEndpoints(motorConfig, outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
        }
#endif
    }
}

float motorConvertFromExternal(uint16_t externalValue)
{
    return motorDevice->vTable.convertExternalToMotor(externalValue);
}

uint16_t motorConvertToExternal(float motorValue)
{
    return motorDevice->vTable.convertMotorToExternal(motorValue);
}

void motorPostInit()
{
    motorDevice->vTable.postInit();
}

void motorPostInitNull(void)
{
}

static bool motorEnableNull(void)
{
    return false;
}

static void motorDisableNull(void)
{
}

static bool motorIsEnabledNull(uint8_t index)
{
    UNUSED(index);

    return false;
}

bool motorUpdateStartNull(void)
{
    return true;
}

void motorWriteNull(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void motorWriteIntNull(uint8_t index, uint16_t value)
{
    UNUSED(index);
    UNUSED(value);
}

void motorUpdateCompleteNull(void)
{
}

static void motorShutdownNull(void)
{
}

static float motorConvertFromExternalNull(uint16_t value)
{
    UNUSED(value);
    return 0.0f ;
}

static uint16_t motorConvertToExternalNull(float value)
{
    UNUSED(value);
    return 0;
}

static const motorVTable_t motorNullVTable = {
    .postInit = motorPostInitNull,
    .enable = motorEnableNull,
    .disable = motorDisableNull,
    .isMotorEnabled = motorIsEnabledNull,
    .updateStart = motorUpdateStartNull,
    .write = motorWriteNull,
    .writeInt = motorWriteIntNull,
    .updateComplete = motorUpdateCompleteNull,
    .convertExternalToMotor = motorConvertFromExternalNull,
    .convertMotorToExternal = motorConvertToExternalNull,
    .shutdown = motorShutdownNull,
};

static motorDevice_t motorNullDevice = {
    .initialized = false,
    .enabled = false,
};

bool isMotorProtocolEnabled(void)
{
    return motorProtocolEnabled;
}

bool isMotorProtocolDshot(void)
{
    return motorProtocolDshot;
}

void motorDevInit(const motorDevConfig_t *motorDevConfig, uint16_t idlePulse, uint8_t motorCount) {
    memset(motors, 0, sizeof(motors));

    bool useUnsyncedPwm = motorDevConfig->useUnsyncedPwm;

    if (isMotorProtocolEnabled()) {
        if (!isMotorProtocolDshot()) {
            motorDevice = motorPwmDevInit(motorDevConfig, idlePulse, motorCount, useUnsyncedPwm);
        }
    }

    if (motorDevice) {
        motorDevice->count = MAX_SUPPORTED_MOTORS;
        motorDevice->initialized = true;
        motorDevice->motorEnableTimeMs = 0;
        motorDevice->enabled = false;
    } else {
        motorNullDevice.vTable = motorNullVTable;
        motorDevice = &motorNullDevice;
    }
}

void motorDisable(void)
{
    motorDevice->vTable.disable();
    motorDevice->enabled = false;
    motorDevice->motorEnableTimeMs = 0;
}

void motorEnable(void)
{
    if (motorDevice->initialized && motorDevice->vTable.enable()) {
        motorDevice->enabled = true;
        motorDevice->motorEnableTimeMs = millis();
    }
}

bool motorIsEnabled(void)
{
    return motorDevice->enabled;
}

bool motorIsMotorEnabled(uint8_t index)
{
    return motorDevice->vTable.isMotorEnabled(index);
}
#endif // USE_MOTOR
