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

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "hw.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "common/boardalignment.h"

#include "compass.h"
#include "compass_qmc5883l.h"
#include "i2c.h"

#ifdef USE_MAG_QMC5883

#ifdef _USE_HW_CLI
static void cliQmc5883l(cli_args_t *args);
#endif

static uint32_t tCal = 0;
static flightDynamicsTrims_t magZeroTempMin;
static flightDynamicsTrims_t magZeroTempMax;

mag_t mag;

#define QMC5883L_MAG_I2C_ADDRESS 0x0D

// Registers
#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A

// data output rates for 5883L
#define QMC5883L_ODR_10HZ  (0x00 << 2)
#define QMC5883L_ODR_50HZ  (0x01 << 2)
#define QMC5883L_ODR_100HZ (0x02 << 2)
#define QMC5883L_ODR_200HZ (0x03 << 2)

// Sensor operation modes
#define QMC5883L_MODE_STANDBY    0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128 (0x10 << 6)
#define QMC5883L_OSR_64  (0x11 << 6)

#define QMC5883L_RST 0x80

#define QMC5883L_REG_DATA_OUTPUT_X 0x00
#define QMC5883L_REG_STATUS 0x06

#define QMC5883L_REG_ID 0x0D
#define QMC5883_ID_VAL 0xFF

static int16_t magADCRaw[XYZ_AXIS_COUNT];
static uint8_t magInit = 0;

void compassConfig_Init(void)
{
    mag.magZero.values.roll = -352;
    mag.magZero.values.pitch = -368;
    mag.magZero.values.yaw = 75;
}

bool compassDetect(magDev_t *magDev, sensor_align_e *alignment)
{
    magSensor_e magHardware = MAG_NONE;

    if (qmc5883lDetect(magDev)) {

        *alignment = MAG_QMC5883;//MAG_QMC5883L_ALIGN;
        magHardware = MAG_QMC5883;
    }

    if (magHardware == MAG_NONE) {
        return false;
    }

    return true;
}

static bool qmc5883lInit(magDev_t *magDev)
{
    bool ack = true;
    uint8_t temp;

    temp = 0x01;
    ack = ack && I2C_ByteWrite_HAL(magDev->address, 0x0B, I2C_MEMADD_SIZE_8BIT, &temp, 1);
    temp = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | QMC5883L_OSR_512 | QMC5883L_RNG_8G;
    ack = ack && I2C_ByteWrite_HAL(magDev->address, QMC5883L_REG_CONF1, 1, &temp, 1);

    if (!ack) {
        return false;
    }

    return true;
}

bool compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)

    sensor_align_e alignment;

    if (!compassDetect(&mag.magDev, &alignment)) {
        return false;
    }

    //LED1_ON;
    mag.magDev.init(&mag.magDev);
    //LED1_OFF;
    magInit = 1;

    #ifdef _USE_HW_CLI
    cliAdd("qmc5883l", cliQmc5883l);
    #endif

    mag.magDev.magAlignment = alignment;

     if (mag.mag_alignment != ALIGN_DEFAULT) {
         mag.magDev.magAlignment = mag.mag_alignment;
     }

    buildRotationMatrixFromAlignment(&mag.mag_customAlignment, &mag.magDev.rotationMatrix);

    return true;
}

static bool qmc5883lRead(magDev_t *magDev, int16_t *magData)
{
    static uint8_t buf[6];
    static uint8_t status;
    static enum {
        STATE_READ_STATUS,
        STATE_WAIT_STATUS,
        STATE_WAIT_READ,
    } state = STATE_READ_STATUS;

    switch (state) {
        default:
        case STATE_READ_STATUS:
            I2C_ByteRead(magDev->address, QMC5883L_REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status, sizeof(status));
            state = STATE_WAIT_STATUS;
            return false;

        case STATE_WAIT_STATUS:
            if ((status & 0x04) == 0) {
                state = STATE_READ_STATUS;
                return false;
            }
            I2C_ByteRead(magDev->address, QMC5883L_REG_DATA_OUTPUT_X, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf));
            state = STATE_WAIT_READ;
            return false;

        case STATE_WAIT_READ:

            magData[X] = (int16_t)(buf[1] << 8 | buf[0]);
            magData[Y] = (int16_t)(buf[3] << 8 | buf[2]);
            magData[Z] = (int16_t)(buf[5] << 8 | buf[4]);

            state = STATE_READ_STATUS;

            return true;
    }

    return false;
}

bool qmc5883lDetect(magDev_t *magDev)
{
    uint8_t temp;
    magDev->address = QMC5883L_MAG_I2C_ADDRESS;

    // Must write reset first  - don't care about the result
    temp = QMC5883L_RST;
    I2C_ByteWrite_HAL(magDev->address, QMC5883L_REG_CONF2, I2C_MEMADD_SIZE_8BIT, &temp, 1);
    delay(20);

    uint8_t sig = 0;
    bool ack = I2C_ByteRead(magDev->address, QMC5883L_REG_ID, I2C_MEMADD_SIZE_8BIT, &sig, 1);
    if (ack && sig == QMC5883_ID_VAL) {
        // Should be in standby mode after soft reset and sensor is really present
        // Reading ChipID of 0xFF alone is not sufficient to be sure the QMC is present
        ack = I2C_ByteRead(magDev->address, QMC5883L_REG_CONF1, I2C_MEMADD_SIZE_8BIT, &sig, 1);
        if (ack && sig != QMC5883L_MODE_STANDBY) {
            return false;
        }
        magDev->init = qmc5883lInit;
        magDev->read = qmc5883lRead;
        return true;
    }

    return false;
}

bool compassIsHealthy(void)
{
    return (mag.magADC[X] != 0) && (mag.magADC[Y] != 0) && (mag.magADC[Z] != 0);
}

void compassStartCalibration(void)
{
    tCal = micros();
    flightDynamicsTrims_t *magZero = &mag.magZero;
    for (int axis = 0; axis < 3; axis++) {
        magZero->raw[axis] = 0;
        magZeroTempMin.raw[axis] = mag.magADC[axis];
        magZeroTempMax.raw[axis] = mag.magADC[axis];
    }
}

bool compassIsCalibrationComplete(void)
{
    return tCal == 0;
}

uint32_t compassUpdate(uint32_t currentTimeUs)
{
    if (!mag.magDev.read(&mag.magDev, magADCRaw)) {
        // No action was taken as the read has not completed
        //schedulerIgnoreTaskExecRate();
        return 1000; // Wait 1ms between states
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mag.magADC[axis] = magADCRaw[axis];
    }
     if (mag.magDev.magAlignment == ALIGN_CUSTOM) {
         alignSensorViaMatrix(mag.magADC, &mag.magDev.rotationMatrix);
     } else {
         alignSensorViaRotation(mag.magADC, mag.magDev.magAlignment);
     }

    flightDynamicsTrims_t *magZero = &mag.magZero;
    if (magInit) {              // we apply offset only once mag calibration is done
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }

    if (tCal != 0) {
        if ((currentTimeUs - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            //LED0_TOGGLE;
            for (int axis = 0; axis < 3; axis++) {
                if (mag.magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = mag.magADC[axis];
                if (mag.magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = mag.magADC[axis];
            }
        } else {
            tCal = 0;
            for (int axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; // Calculate offsets
            }

            //saveConfigAndNotify();
        }
    }

    return TASK_PERIOD_HZ(10);
}

#ifdef _USE_HW_CLI
void cliQmc5883l(cli_args_t *args)
{
  bool ret = false;

if (args->argc == 1 && args->isStr(0, "compass_show") == true)
{
    uint32_t pre_time;
    pre_time = millis();
    while(cliKeepLoop())
    {
        if (millis()-pre_time >= 1000)
        {
            pre_time = millis();
            // cliPrintf("acc x: %d, y: %d, z: %d\n\r", x, y, z);
        }
    }
    ret = true;
    }

  if (args->argc == 3 && args->isStr(0, "mem_read") == true)
  {
    uint8_t ch;
    uint8_t addr;

    ch   = (uint8_t)args->getData(1);
    addr = (uint8_t)args->getData(2);
    addr |= 0x80;

    ret = true;
  }

    if (args->argc == 4 && args->isStr(0, "mem_write") == true)
  {
    uint8_t ch;
    uint8_t addr;

    ch     = (uint8_t)args->getData(1);
    addr   = (uint8_t)args->getData(2);

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("qmc5883l show \n\r");
    cliPrintf("qmc5883l mem_read ch0:1, addr \n\r");
    cliPrintf("qmc5883l mem_write ch0:1, addr data \n\r");
  }
}
#endif
#endif
