/*
 * AT24C08.h
 *
 *  Created on: 2019. 12. 4.
 *      Author: Administrator
 */

#ifndef INC_PARAMETER_H_
#define INC_PARAMETER_H_

#include "hw.h"

typedef union _Parser
{
	unsigned char byte[4];
	float f;
}Parser;

void SDCARD_Page_Write(uint32_t page, uint8_t* data, uint32_t len);
void SDCARD_Page_Read(uint32_t page, uint8_t* data, uint32_t len);
void EP_PIDGain_Write(unsigned char id, float PGain, float IGain, float DGain);
unsigned char EP_PIDGain_Read(unsigned char id, float* PGain, float* IGain, float* DGain);
void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf);
void Encode_Msg_GPS(unsigned char* telemetry_tx_buf);
void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d);
void pidConfig_Init(void);


#endif /* INC_PARAMETER_H_ */
