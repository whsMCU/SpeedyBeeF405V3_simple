/*
 * accgyro.h
 *
 *  Created on: 2023. 10. 5.
 *      Author: 왕학승
 */

#ifndef SRC_MAIN_DRIVERS_ACCGYRO_ACCGYRO_H_
#define SRC_MAIN_DRIVERS_ACCGYRO_ACCGYRO_H_

#include "common/axis.h"
#include "common/maths.h"

typedef enum {
    GYRO_HARDWARE_LPF_NORMAL,
    GYRO_HARDWARE_LPF_OPTION_1,
    GYRO_HARDWARE_LPF_OPTION_2,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    GYRO_HARDWARE_LPF_EXPERIMENTAL,
#endif
    GYRO_HARDWARE_LPF_COUNT
} gyroHardwareLpf_e;

typedef struct gyro_s {
    float scale;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];     // aligned, calibrated, scaled, but unfiltered data from the sensor(s)
    float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data
    float gyroZero[XYZ_AXIS_COUNT];
    uint16_t calibratingG;
    stdev_t var[XYZ_AXIS_COUNT];
    float accumulatedMeasurements[XYZ_AXIS_COUNT];
    float gyroPrevious[XYZ_AXIS_COUNT];
    int accumulatedMeasurementCount;
    volatile bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint16_t gyroSampleRateHz;
    uint32_t sampleLooptime;
	uint32_t targetLooptime;
} gyro_t;

typedef struct acc_s {
    float acc_1G_rec;
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    volatile bool dataReady;
    bool acc_high_fsr;
    uint16_t sampleRateHz;
    float accADC[XYZ_AXIS_COUNT];
    float accADCf[XYZ_AXIS_COUNT];
    float accZero[XYZ_AXIS_COUNT];
    int accumulatedMeasurementCount;
    float accumulatedMeasurements[XYZ_AXIS_COUNT];
    uint16_t calibratingA;
    bool isAccelUpdatedAtLeastOnce;
} acc_t;

typedef struct mpu_s {
	acc_t acc;
	gyro_t gyro;
    uint8_t *txBuf, *rxBuf;
    uint16_t sampleRateHz;
    uint32_t sampleLooptime;
} mpu_t;


#endif /* SRC_MAIN_DRIVERS_ACCGYRO_ACCGYRO_H_ */
