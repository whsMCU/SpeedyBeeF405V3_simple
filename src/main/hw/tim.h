/*
 * timer.h
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_TIMER_H_
#define SRC_COMMON_HW_INCLUDE_TIMER_H_

#include "hw.h"

#ifdef _USE_HW_TIMER

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

bool TIM_Init(void);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_TIMER_H_ */
