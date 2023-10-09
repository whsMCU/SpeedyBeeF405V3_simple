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
#include "common/filter.h"

typedef enum {
    GYRO_HARDWARE_LPF_NORMAL,
    GYRO_HARDWARE_LPF_OPTION_1,
    GYRO_HARDWARE_LPF_OPTION_2,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    GYRO_HARDWARE_LPF_EXPERIMENTAL,
#endif
    GYRO_HARDWARE_LPF_COUNT
} gyroHardwareLpf_e;

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t calibrationCompleted;
} flightDynamicsTrims_def_t;

typedef union flightDynamicsTrims_u {
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

enum {
    FILTER_LPF1 = 0,
    FILTER_LPF2
};

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;

typedef struct gyro_s {
    bool downsampleFilterEnabled;      // if true then downsample using gyro lowpass 2, otherwise use averaging
    uint8_t sampleCount;               // gyro sensor sample counter
    float sampleSum[XYZ_AXIS_COUNT];   // summed samples used for downsampling

    uint16_t gyro_lpf1_static_hz;
    uint16_t gyro_lpf2_static_hz;

    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;

    // Lowpass primary/secondary
    uint8_t gyro_lpf1_type;
    uint8_t gyro_lpf2_type;

    uint16_t gyro_lpf1_dyn_min_hz;
    uint16_t gyro_lpf1_dyn_max_hz;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

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
	uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    uint16_t accLpfCutHz;
    biquadFilter_t accFilter[XYZ_AXIS_COUNT];
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
