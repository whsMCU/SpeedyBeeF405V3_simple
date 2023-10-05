#ifndef SRC_COMMON_HW_INCLUDE_BMI270_H_
#define SRC_COMMON_HW_INCLUDE_BMI270_H_

#include "hw.h"
#include "accgyro.h"

bool bmi270_Init(void);
bool bmi270Detect(uint8_t ch);

void bmi270Config(void);
bool bmi270SpiAccRead(void);
bool bmi270SpiGyroRead(void);
void bmi270SetCallBack(void (*p_func)(void));
uint8_t bmi270InterruptStatus(void);
void bmi270Intcallback(void);
void gyroUpdate(void);
void accUpdate(void);

#endif /* SRC_COMMON_HW_INCLUDE_BMI270_H_ */
