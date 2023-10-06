#ifndef SRC_COMMON_HW_INCLUDE_DSP310_H_
#define SRC_COMMON_HW_INCLUDE_DSP310_H_

#include "hw.h"
#include "drivers/barometer/barometer.h"

extern baro_t baro;

void dps310_Init(void);
bool dps310Detect(baroDev_t *baro);
bool dps310AddrRead(void);
bool dps310Read(baroDev_t *baro);
bool dps310SetCallBack(void (*p_func)(void));
uint32_t baroUpdate(uint32_t currentTimeUs);
bool baroIsCalibrationComplete(void);
void performBaroCalibrationCycle(void);
int32_t baroCalculateAltitude(void);

#endif /* SRC_COMMON_HW_INCLUDE_DSP310_H_ */
