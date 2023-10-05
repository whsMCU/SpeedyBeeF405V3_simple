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

#include "hw.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/bmi270.h"

mpu_t mpu;

#ifdef USE_ACCGYRO_BMI270

#define CALIBRATING_ACC_CYCLES              400
#define acc_lpf_factor 4

#define gyroMovementCalibrationThreshold    48

#define GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_4000DPS (4000.0f / (1 << 15))   //  8.192 dps/lsb scalefactor for 4000dps sensors

// 10 MHz max SPI frequency
#define BMI270_MAX_SPI_CLK_HZ 10000000

#define BMI270_FIFO_FRAME_SIZE 6

#define BMI270_CONFIG_SIZE 328

const uint8_t bmi270_maximum_fifo_config_file[BMI270_CONFIG_SIZE] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

#define BMI270_CHIP_ID 0x24

// BMI270 registers (not the complete list)
typedef enum {
    BMI270_REG_CHIP_ID = 0x00,
    BMI270_REG_ERR_REG = 0x02,
    BMI270_REG_STATUS = 0x03,
    BMI270_REG_ACC_DATA_X_LSB = 0x0C,
    BMI270_REG_GYR_DATA_X_LSB = 0x12,
    BMI270_REG_SENSORTIME_0 = 0x18,
    BMI270_REG_SENSORTIME_1 = 0x19,
    BMI270_REG_SENSORTIME_2 = 0x1A,
    BMI270_REG_EVENT = 0x1B,
    BMI270_REG_INT_STATUS_0 = 0x1C,
    BMI270_REG_INT_STATUS_1 = 0x1D,
    BMI270_REG_INTERNAL_STATUS = 0x21,
    BMI270_REG_TEMPERATURE_LSB = 0x22,
    BMI270_REG_TEMPERATURE_MSB = 0x23,
    BMI270_REG_FIFO_LENGTH_LSB = 0x24,
    BMI270_REG_FIFO_LENGTH_MSB = 0x25,
    BMI270_REG_FIFO_DATA = 0x26,
    BMI270_REG_ACC_CONF = 0x40,
    BMI270_REG_ACC_RANGE = 0x41,
    BMI270_REG_GYRO_CONF = 0x42,
    BMI270_REG_GYRO_RANGE = 0x43,
    BMI270_REG_AUX_CONF = 0x44,
    BMI270_REG_FIFO_DOWNS = 0x45,
    BMI270_REG_FIFO_WTM_0 = 0x46,
    BMI270_REG_FIFO_WTM_1 = 0x47,
    BMI270_REG_FIFO_CONFIG_0 = 0x48,
    BMI270_REG_FIFO_CONFIG_1 = 0x49,
    BMI270_REG_SATURATION = 0x4A,
    BMI270_REG_INT1_IO_CTRL = 0x53,
    BMI270_REG_INT2_IO_CTRL = 0x54,
    BMI270_REG_INT_LATCH = 0x55,
    BMI270_REG_INT1_MAP_FEAT = 0x56,
    BMI270_REG_INT2_MAP_FEAT = 0x57,
    BMI270_REG_INT_MAP_DATA = 0x58,
    BMI270_REG_INIT_CTRL = 0x59,
    BMI270_REG_INIT_DATA = 0x5E,
    BMI270_REG_ACC_SELF_TEST = 0x6D,
    BMI270_REG_GYR_SELF_TEST_AXES = 0x6E,
    BMI270_REG_PWR_CONF = 0x7C,
    BMI270_REG_PWR_CTRL = 0x7D,
    BMI270_REG_CMD = 0x7E,
} bmi270Register_e;

// BMI270 register configuration values
typedef enum {
    BMI270_VAL_CMD_SOFTRESET = 0xB6,
    BMI270_VAL_CMD_FIFOFLUSH = 0xB0,
    BMI270_VAL_PWR_CTRL = 0x0E,              // enable gyro, acc and temp sensors
    BMI270_VAL_PWR_CONF = 0x02,              // disable advanced power save, enable FIFO self-wake
    BMI270_VAL_ACC_CONF_ODR800 = 0x0B,       // set acc sample rate to 800hz
    BMI270_VAL_ACC_CONF_ODR1600 = 0x0C,      // set acc sample rate to 1600hz
    BMI270_VAL_ACC_CONF_BWP = 0x02,          // set acc filter in normal mode
    BMI270_VAL_ACC_CONF_HP = 0x01,           // set acc in high performance mode
    BMI270_VAL_ACC_RANGE_8G = 0x02,          // set acc to 8G full scale
    BMI270_VAL_ACC_RANGE_16G = 0x03,         // set acc to 16G full scale
    BMI270_VAL_GYRO_CONF_ODR3200 = 0x0D,     // set gyro sample rate to 3200hz
    BMI270_VAL_GYRO_CONF_BWP_OSR4 = 0x00,    // set gyro filter in OSR4 mode
    BMI270_VAL_GYRO_CONF_BWP_OSR2 = 0x01,    // set gyro filter in OSR2 mode
    BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,    // set gyro filter in normal mode
    BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,  // set gyro in high performance noise mode
    BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01, // set gyro in high performance filter mode

    BMI270_VAL_GYRO_RANGE_2000DPS = 0x08,    // set gyro to 2000dps full scale
                                             // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
                                             // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)

    BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04,// enable the data ready interrupt pin 1
    BMI270_VAL_INT_MAP_FIFO_WM_INT1 = 0x02,  // enable the FIFO watermark interrupt pin 1
    BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A,  // active high, push-pull, output enabled, input disabled 
    BMI270_VAL_FIFO_CONFIG_0 = 0x00,         // don't stop when full, disable sensortime frame
    BMI270_VAL_FIFO_CONFIG_1 = 0x80,         // only gyro data in FIFO, use headerless mode
    BMI270_VAL_FIFO_DOWNS = 0x00,            // select unfiltered gyro data with no downsampling (6.4KHz samples)
    BMI270_VAL_FIFO_WTM_0 = 0x06,            // set the FIFO watermark level to 1 gyro sample (6 bytes)
    BMI270_VAL_FIFO_WTM_1 = 0x00,            // FIFO watermark MSB
} bmi270ConfigValues_e;

static uint8_t dev = BMI270;

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

static uint8_t _buffer[16] = {0, };

#define GYRO_BUF_SIZE 32

static uint8_t gyroBuf[GYRO_BUF_SIZE];

// BMI270 register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.
static uint8_t bmi270RegisterRead(uint8_t dev, bmi270Register_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void bmi270RegisterWrite(uint8_t dev, bmi270Register_e registerId, uint8_t value, uint8_t delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

#ifdef _USE_HW_CLI
static void cliBmi270(cli_args_t *args);
#endif

// Toggle the CS to switch the device into SPI mode.
// Device switches initializes as I2C and switches to SPI on a low to high CS transition
static void bmi270EnableSPI(uint8_t dev)
{
	UNUSED(dev);
    gpioPinWrite(_PIN_BMI270_CS, _DEF_LOW);
    delay(1);
    gpioPinWrite(_PIN_BMI270_CS, _DEF_HIGH);
    delay(10);
}

static void bmi270UploadConfig(uint8_t dev)
{
    bmi270RegisterWrite(dev, BMI270_REG_PWR_CONF, 0, 1);
    bmi270RegisterWrite(dev, BMI270_REG_INIT_CTRL, 0, 1);

    // Transfer the config file
    spiWriteRegBuf(dev, BMI270_REG_INIT_DATA, (uint8_t *)bmi270_maximum_fifo_config_file, sizeof(bmi270_maximum_fifo_config_file));

    delay(10);
    bmi270RegisterWrite(dev, BMI270_REG_INIT_CTRL, 1, 1);
}

static uint8_t getBmiOsrMode()
{
    switch(mpu.gyro.hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMI270_VAL_GYRO_CONF_BWP_OSR4;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return BMI270_VAL_GYRO_CONF_BWP_OSR2;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return BMI270_VAL_GYRO_CONF_BWP_NORM;
//        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
//            return BMI270_VAL_GYRO_CONF_BWP_NORM;
    }
    return 0;
}

void bmi270Config()
{
    // If running in hardware_lpf experimental mode then switch to FIFO-based,
    // 6.4KHz sampling, unfiltered data vs. the default 3.2KHz with hardware filtering
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    const bool fifoMode = (gyro.gyroSensor1.gyroDev.hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL);
#else
    const bool fifoMode = false;
#endif

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi270RegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_SOFTRESET, 100);
    // Toggle the chip into SPI mode
    bmi270EnableSPI(dev);

    bmi270UploadConfig(dev);

    // Configure the FIFO
    if (fifoMode) {
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_CONFIG_0, BMI270_VAL_FIFO_CONFIG_0, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_CONFIG_1, BMI270_VAL_FIFO_CONFIG_1, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_DOWNS, BMI270_VAL_FIFO_DOWNS, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_WTM_0, BMI270_VAL_FIFO_WTM_0, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_WTM_1, BMI270_VAL_FIFO_WTM_1, 1);
    }

    // Configure the accelerometer
	bmi270RegisterWrite(dev, BMI270_REG_ACC_CONF, (BMI270_VAL_ACC_CONF_HP << 7) | (BMI270_VAL_ACC_CONF_BWP << 4) | BMI270_VAL_ACC_CONF_ODR800, 1);

	// Configure the accelerometer full-scale range
	bmi270RegisterWrite(dev, BMI270_REG_ACC_RANGE, BMI270_VAL_ACC_RANGE_16G, 1);

	// Configure the gyro
	bmi270RegisterWrite(dev, BMI270_REG_GYRO_CONF, (BMI270_VAL_GYRO_CONF_FILTER_PERF << 7) | (BMI270_VAL_GYRO_CONF_NOISE_PERF << 6) | (getBmiOsrMode() << 4) | BMI270_VAL_GYRO_CONF_ODR3200, 1);

	// Configure the gyro full-range scale
	bmi270RegisterWrite(dev, BMI270_REG_GYRO_RANGE, BMI270_VAL_GYRO_RANGE_2000DPS, 1);

	// Configure the gyro data ready interrupt
	if (fifoMode) {
		// Interrupt driven by FIFO watermark level
		bmi270RegisterWrite(dev, BMI270_REG_INT_MAP_DATA, BMI270_VAL_INT_MAP_FIFO_WM_INT1, 1);
	} else {
		// Interrupt driven by data ready
		bmi270RegisterWrite(dev, BMI270_REG_INT_MAP_DATA, BMI270_VAL_INT_MAP_DATA_DRDY_INT1, 1);
	}

	// Configure the behavior of the INT1 pin
	bmi270RegisterWrite(dev, BMI270_REG_INT1_IO_CTRL, BMI270_VAL_INT1_IO_CTRL_PINMODE, 1);

	// Configure the device for  performance mode
	bmi270RegisterWrite(dev, BMI270_REG_PWR_CONF, BMI270_VAL_PWR_CONF, 1);

	// Enable the gyro, accelerometer and temperature sensor - disable aux interface
	bmi270RegisterWrite(dev, BMI270_REG_PWR_CTRL, BMI270_VAL_PWR_CTRL, 1);

	// Flush the FIFO
	if (fifoMode) {
		bmi270RegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_FIFOFLUSH, 1);
	}
}

bool bmi270_Init(void)
{
    bool ret = true;

    bmi270EnableSPI(dev);
    // Allow 100ms before attempting to access gyro's SPI bus
    // Do this once here rather than in each detection routine to speed boot
    while (millis() < 100);
    delay(35);

	for(int i = 0; i < 5; i++)
	{
		if (bmi270Detect(_DEF_SPI1))
		{
			//bmi270Config();

			break;
		}
		delay(100);
	}

    // SPI DMA buffer required per device
    mpu.txBuf = gyroBuf;
    mpu.rxBuf = &gyroBuf[GYRO_BUF_SIZE / 2];

    mpu.gyro.gyro_high_fsr = false;
    mpu.gyro.hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
	mpu.gyro.gyroSampleRateHz = 3200;
	mpu.gyro.sampleLooptime = 1e6 / mpu.gyro.gyroSampleRateHz;
	mpu.gyro.targetLooptime = 1e6 / mpu.gyro.gyroSampleRateHz;
	mpu.gyro.calibratingG = 4000;
	mpu.gyro.scale = GYRO_SCALE_2000DPS;

    bmi270Config();

	//gyro.scale = gyro.gyroSensor1.gyroDev.scale;

	mpu.acc.acc_high_fsr = false;

	mpu.acc.accZero[X] = 29;
	mpu.acc.accZero[Y] = -35;
	mpu.acc.accZero[Z] = -9;
	mpu.acc.sampleRateHz = 800;
    mpu.acc.acc_1G = 512 * 4;   // 16G sensor scale
    mpu.acc.acc_1G_rec = 1.0f / mpu.acc.acc_1G;


    #ifdef _USE_HW_CLI
        cliAdd("bmi270", cliBmi270);
    #endif

    return ret;
}

bool bmi270Detect(uint8_t ch)
{
	UNUSED(ch);
	bmi270EnableSPI(dev);
	if (bmi270RegisterRead(dev, BMI270_REG_CHIP_ID) == BMI270_CHIP_ID)
	{
		return true;
	}
	return false;
}

/*
 * Gyro interrupt service routine
 */

// Called in ISR context
// Gyro read has just completed
void bmi270Intcallback(void)
{
	static uint32_t pre_time = 0;
	static uint32_t rx_callback_dt = 0;
	rx_callback_dt = micros() - pre_time;
    pre_time = micros();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t pre_time = 0;
	static uint32_t exit_callback_dt = 0;
    if(GPIO_Pin==GPIO_PIN_4)
    {
		exit_callback_dt = micros() - pre_time;
		pre_time = micros();
		//SPI_ByteReadWrite_DMA(dev, gyro_temp->txBuf, gyro_temp->rxBuf, 14);
		spiReadWriteBuf(dev, mpu.txBuf, mpu.rxBuf, 14);
    }
}

bool bmi270SpiAccRead(void)
{
	// If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
	// up an old value.

	// This data was read from the gyro, which is the same SPI device as the acc
	uint16_t *mpuData = (uint16_t *)(mpu.rxBuf+1);
	mpu.acc.ADCRaw[X] = mpuData[0];
	mpu.acc.ADCRaw[Y] = mpuData[1];
	mpu.acc.ADCRaw[Z] = mpuData[2];
  return true;
}

static bool bmi270GyroReadRegister(void)
{
    uint16_t *mpuData = (uint16_t *)(mpu.rxBuf+1);
	// Initialise the tx buffer to all 0x00
	memset(mpu.txBuf, 0x00, 16);
	// Using DMA for gyro access upsets the scheduler on the F4
	mpu.txBuf[0] = BMI270_REG_ACC_DATA_X_LSB | 0x80;

	// If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
	// up an old value.
	mpu.gyro.ADCRaw[X] = mpuData[3];
	mpu.gyro.ADCRaw[Y] = mpuData[4];
	mpu.gyro.ADCRaw[Z] = mpuData[5];

    return true;
}

bool bmi270SpiGyroRead(void)
{
	// running in 3.2KHz register mode
	return bmi270GyroReadRegister();
}

void performGyroCalibration(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (mpu.gyro.calibratingG == 4000) {
            mpu.gyro.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        mpu.gyro.gyroZero[axis] += mpu.gyro.ADCRaw[axis];
        devPush(&mpu.gyro.var[axis], mpu.gyro.ADCRaw[axis]);

        if (mpu.gyro.calibratingG == 1) {
            const float stddev = devStandardDeviation(&mpu.gyro.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
            	mpu.gyro.calibratingG = 4000;
                return;
            }

            // please take care with exotic boardalignment !!
            mpu.gyro.gyroZero[axis] = mpu.gyro.gyroZero[axis] / 4000;
//            if (axis == Z) {
//              mpu.gyro.gyroZero[axis] -= ((float)gyroConfig.gyro_offset_yaw / 100);
//            }
        }
    }
    --mpu.gyro.calibratingG;
}

void gyroUpdate(void)
{
	bmi270SpiGyroRead();
	if(mpu.gyro.calibratingG == 0)
	{
	    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
	    	mpu.gyro.ADCRaw[axis] -= mpu.gyro.gyroZero[axis];
	    	mpu.gyro.gyroADC[axis] = mpu.gyro.ADCRaw[axis] * mpu.gyro.scale;

            // integrate using trapezium rule to avoid bias
            mpu.gyro.accumulatedMeasurements[axis] += 0.5f * (mpu.gyro.gyroPrevious[axis] + mpu.gyro.gyroADC[axis]) * mpu.gyro.targetLooptime;
            mpu.gyro.gyroPrevious[axis] = mpu.gyro.gyroADCf[axis];
	    }
        mpu.gyro.accumulatedMeasurementCount++;
	}else {
		performGyroCalibration();
	}
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (mpu.gyro.accumulatedMeasurementCount) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        const uint32_t accumulatedMeasurementTimeUs = mpu.gyro.accumulatedMeasurementCount * mpu.gyro.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = mpu.gyro.accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            mpu.gyro.accumulatedMeasurements[axis] = 0.0f;
        }
        mpu.gyro.accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

void accUpdate(void)
{
	static int32_t a[3];

	bmi270SpiAccRead();
	mpu.acc.isAccelUpdatedAtLeastOnce = true;

	if(mpu.acc.calibratingA>0)
	{
		for(int axis=0; axis <XYZ_AXIS_COUNT; axis++)
		{
			// Reset a[axis] at start of calibration
			if (mpu.acc.calibratingA == 512) a[axis] = 0;
			// Sum up 512 readings
			a[axis] +=mpu.acc.ADCRaw[axis];
			// Clear global variables for next reading
			mpu.acc.ADCRaw[axis] = 0;
			mpu.acc.accZero[axis] = 0;
		}
		// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
		if (mpu.acc.calibratingA == 1)
		{
			mpu.acc.accZero[X]  = a[X]>>9;
			mpu.acc.accZero[Y]  = a[Y]>>9;
			mpu.acc.accZero[Z]  = (a[Z]>>9) - mpu.acc.acc_1G;
			f.CALIBRATE_ACC = 0;
		}
		mpu.acc.calibratingA--;
	}

	if(mpu.acc.calibratingA == 0)
	{
	    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
	    	mpu.acc.ADCRaw[axis] -=  mpu.acc.accZero[axis];
			if (acc_lpf_factor > 0)
			{
				mpu.acc.accADC[axis] = mpu.acc.accADC[axis] * (1.0f - (1.0f / acc_lpf_factor)) + mpu.acc.ADCRaw[axis] * (1.0f / acc_lpf_factor);
				mpu.acc.accADCf[axis] = mpu.acc.accADC[axis];
			}
	    }
	}
    ++mpu.acc.accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mpu.acc.accumulatedMeasurements[axis] += mpu.acc.accADCf[axis];
    }

}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (mpu.acc.accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = mpu.acc.accumulatedMeasurements[axis] / mpu.acc.accumulatedMeasurementCount;
            mpu.acc.accumulatedMeasurements[axis] = 0.0f;
        }
        mpu.acc.accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

static void (*frameCallBack)(void) = NULL;

void bmi270SetCallBack(void (*p_func)(void))
{
  frameCallBack = p_func;
}

uint8_t bmi270InterruptStatus(void)
{
    return bmi270RegisterRead(dev, BMI270_REG_INT_STATUS_1);
}

#ifdef _USE_HW_CLI
void cliBmi270(cli_args_t *args)
{
  bool ret = false;

if (args->argc == 1 && args->isStr(0, "gyro_show") == true)
  {
    uint32_t pre_time;
 	pre_time = millis();
    int16_t x=0, y=0, z=0;
    while(cliKeepLoop())
    {
        if (millis()-pre_time >= 1000)
    	{
     		pre_time = millis();
			memset(_buffer, 0x00, 7);
            SPI_ByteRead(_DEF_SPI1, (BMI270_REG_GYR_DATA_X_LSB | 0x80), _buffer, 7);
            x = (uint16_t)_buffer[2]<<8 | (uint16_t)_buffer[1];
            y = (uint16_t)_buffer[4]<<8 | (uint16_t)_buffer[3];
            z = (uint16_t)_buffer[6]<<8 | (uint16_t)_buffer[5];
            cliPrintf("gyro x: %d, y: %d, z: %d\n\r", x, y, z);
    	}
    }
    ret = true;
  }

if (args->argc == 1 && args->isStr(0, "acc_show") == true)
{
    uint32_t pre_time;
    pre_time = millis();
    int16_t x=0, y=0, z=0;
while(cliKeepLoop())
{
    if (millis()-pre_time >= 1000)
    {
        pre_time = millis();
        memset(_buffer, 0x00, 7);
        SPI_ByteRead(_DEF_SPI1, (BMI270_REG_ACC_DATA_X_LSB | 0x80), _buffer, 7);
        x = (uint16_t)_buffer[2]<<8 | (uint16_t)_buffer[1];
        y = (uint16_t)_buffer[4]<<8 | (uint16_t)_buffer[3];
        z = (uint16_t)_buffer[6]<<8 | (uint16_t)_buffer[5];
        cliPrintf("acc x: %d, y: %d, z: %d\n\r", x, y, z);
    }
}
ret = true;
}

  if (args->argc == 3 && args->isStr(0, "mem_read") == true)
  {
    uint8_t ch;
    uint8_t addr;
    uint8_t buffer[2] = {0, 0};
    HAL_StatusTypeDef status;

    ch   = (uint8_t)args->getData(1);
    addr = (uint8_t)args->getData(2);
    addr |= 0x80;

    status = SPI_ByteRead(ch, addr, buffer, 2);

    if(status == HAL_OK)
    {
        cliPrintf("bmi270 mem_read : ch (%d), addr (0x%X), data[0] : (0x%X), data[1] : (0x%X), status (%d)\n", ch, addr, buffer[0], buffer[1], status);
    }else
    {
        cliPrintf("bmi270 read - Fail(%d) \n", status);
    }
    ret = true;
  }

    if (args->argc == 4 && args->isStr(0, "mem_write") == true)
  {
    uint8_t ch;
    uint8_t addr;
    uint8_t buffer;
    HAL_StatusTypeDef status;

    ch     = (uint8_t)args->getData(1);
    addr   = (uint8_t)args->getData(2);
    buffer = (uint8_t)args->getData(3);

    status = SPI_ByteWrite(ch, addr, &buffer, 1);

    if(status == HAL_OK)
    {
        cliPrintf("bmi270 mem_write : ch (%d), addr (0x%X), data : (0x%X), status (%d)\n", ch, addr, buffer, status);
    }else
    {
        cliPrintf("bmi270 write - Fail(%d) \n", status);
    }
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("bmi270 gyro_show \n\r");
    cliPrintf("bmi270 acc_show \n\r");
    cliPrintf("bmi270 mem_read ch0:1, addr \n\r");
    cliPrintf("bmi270 mem_write ch0:1, addr data \n\r");
  }
}
#endif



#endif // USE_ACCGYRO_BMI270
