/*
 * mixer.h
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_MIXER_H_
#define SRC_COMMON_CORE_MIXER_H_

#include "hw.h"

#include "common/time.h"


#define QuadP 0
#define QuadX 1

extern int16_t motor[4];


void mixerInit(void);
void mixTable(timeUs_t currentTimeUs);


#endif /* SRC_COMMON_CORE_MIXER_H_ */
