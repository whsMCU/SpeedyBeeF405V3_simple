/*
 * AT24C08.c
 *
 *  Created on: 2019. 12. 4.
 *      Author: Administrator
 */

#include <parameter.h>
#include "hw/sd.h"
#include "hw/fatfs.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "accgyro/bmi270.h"
#include "barometer/barometer_dps310.h"
#include "compass/compass_qmc5883l.h"

void SDCARD_Page_Write(uint32_t page, uint8_t* data, uint32_t len)
{
	uint32_t sdAddress = page * 16;

	sdWriteBlocks(sdAddress, data, len, 100);
}

void SDCARD_Page_Read(uint32_t page, uint8_t* data, uint32_t len)
{
	uint32_t sdAddress = page * 16;

	sdReadBlocks(sdAddress, (uint8_t *)data, len, 100);
}

void EP_PIDGain_Write(unsigned char id, float PGain, float IGain, float DGain)
{
	uint8_t buf_write[16];
	Parser parser;

	buf_write[0] = 0x45;
	buf_write[1] = 0x50;
	buf_write[2] = id;
	parser.f = PGain;
	buf_write[3] = parser.byte[0];
	buf_write[4] = parser.byte[1];
	buf_write[5] = parser.byte[2];
	buf_write[6] = parser.byte[3];

	parser.f = IGain;
	buf_write[7] = parser.byte[0];
	buf_write[8] = parser.byte[1];
	buf_write[9] = parser.byte[2];
	buf_write[10] = parser.byte[3];

	parser.f = DGain;
	buf_write[11] = parser.byte[0];
	buf_write[12] = parser.byte[1];
	buf_write[13] = parser.byte[2];
	buf_write[14] = parser.byte[3];

	uint8_t chksum = 0xff;
	for(int i=0;i<15;i++) chksum -= buf_write[i];

	buf_write[15] = chksum;

	switch(id)
	{
	case 0:
		SDCARD_Page_Write(0, &buf_write[0], 16);
		break;
	case 1:
		SDCARD_Page_Write(1, &buf_write[0], 16);
		break;
	case 2:
		SDCARD_Page_Write(2, &buf_write[0], 16);
		break;
	case 3:
		SDCARD_Page_Write(3, &buf_write[0], 16);
		break;
	case 4:
		SDCARD_Page_Write(4, &buf_write[0], 16);
		break;
	case 5:
		SDCARD_Page_Write(5, &buf_write[0], 16);
		break;
	}
}

unsigned char EP_PIDGain_Read(unsigned char id, float* PGain, float* IGain, float* DGain)
{
	uint8_t buf_read[16];
	Parser parser;

	switch(id)
	{
	case 0:
		SDCARD_Page_Read(0, &buf_read[0], 16);
		break;
	case 1:
		SDCARD_Page_Read(1, &buf_read[0], 16);
		break;
	case 2:
		SDCARD_Page_Read(2, &buf_read[0], 16);
		break;
	case 3:
		SDCARD_Page_Read(3, &buf_read[0], 16);
		break;
	case 4:
		SDCARD_Page_Read(4, &buf_read[0], 16);
		break;
	case 5:
		SDCARD_Page_Read(5, &buf_read[0], 16);
		break;
	}

	uint8_t chksum = 0xff;
	for(int i=0;i<15;i++) chksum -= buf_read[i];

	if(buf_read[15] == chksum && buf_read[0] == 0x45 && buf_read[1] == 0x50)
	{
		parser.byte[0] = buf_read[3];
		parser.byte[1] = buf_read[4];
		parser.byte[2] = buf_read[5];
		parser.byte[3] = buf_read[6];
		*PGain = parser.f;

		parser.byte[0] = buf_read[7];
		parser.byte[1] = buf_read[8];
		parser.byte[2] = buf_read[9];
		parser.byte[3] = buf_read[10];
		*IGain = parser.f;

		parser.byte[0] = buf_read[11];
		parser.byte[1] = buf_read[12];
		parser.byte[2] = buf_read[13];
		parser.byte[3] = buf_read[14];
		*DGain = parser.f;

		return 0;
	}

	return 1;
}

uint8_t telemetry_tx_buf[40];
uint8_t telemetry_rx_buf[20];

void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;

	  telemetry_tx_buf[2] = 0x10;

	  telemetry_tx_buf[3] = (short)(attitude.values.roll*10);
	  telemetry_tx_buf[4] = ((short)(attitude.values.roll*10))>>8;

	  telemetry_tx_buf[5] = (short)(attitude.values.pitch*10);
	  telemetry_tx_buf[6] = ((short)(attitude.values.pitch*10))>>8;

	  telemetry_tx_buf[7] = (unsigned short)(attitude.values.yaw*10);
	  telemetry_tx_buf[8] = ((unsigned short)(attitude.values.yaw*10))>>8;

	  telemetry_tx_buf[9] = (short)(baro.BaroAlt*10);
	  telemetry_tx_buf[10] = ((short)(baro.BaroAlt*10))>>8;

	  telemetry_tx_buf[11] = 0x00;//(short)((iBus.RH-1500)*0.1f*100);
	  telemetry_tx_buf[12] = 0x00;//((short)((iBus.RH-1500)*0.1f*100))>>8;

	  telemetry_tx_buf[13] = 0x00;//(short)((iBus.RV-1500)*0.1f*100);
	  telemetry_tx_buf[14] = 0x00;//((short)((iBus.RV-1500)*0.1f*100))>>8;

	  telemetry_tx_buf[15] = 0x00;//(unsigned short)((iBus.LH-1000)*0.36f*100);
	  telemetry_tx_buf[16] = 0x00;//((unsigned short)((iBus.LH-1000)*0.36f*100))>>8;

	  telemetry_tx_buf[17] = 0x00;//(short)(iBus.LV*10);
	  telemetry_tx_buf[18] = 0x00;//((short)(iBus.LV*10))>>8;

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

void Encode_Msg_GPS(unsigned char* telemetry_tx_buf)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;

	  telemetry_tx_buf[2] = 0x11;

	  telemetry_tx_buf[3] = 0x00;//posllh.lat;
	  telemetry_tx_buf[4] = 0x00;//posllh.lat>>8;
	  telemetry_tx_buf[5] = 0x00;//posllh.lat>>16;
	  telemetry_tx_buf[6] = 0x00;//posllh.lat>>24;

	  telemetry_tx_buf[7] = 0x00;//posllh.lon;
	  telemetry_tx_buf[8] = 0x00;//posllh.lon>>8;
	  telemetry_tx_buf[9] = 0x00;//posllh.lon>>16;
	  telemetry_tx_buf[10] = 0x00;//posllh.lon>>24;

	  telemetry_tx_buf[11] = 0x00;//(unsigned short)(batVolt*100);
	  telemetry_tx_buf[12] = 0x00;//((unsigned short)(batVolt*100))>>8;

	  telemetry_tx_buf[13] = 0x00;//iBus.SwA == 1000 ? 0 : 1;
	  telemetry_tx_buf[14] = 0x00;//iBus.SwC == 1000 ? 0 : iBus.SwC == 1500 ? 1 : 2;

	  telemetry_tx_buf[15] = 0x00;//failsafe_flag;

	  telemetry_tx_buf[16] = 0x00;
	  telemetry_tx_buf[17] = 0x00;
	  telemetry_tx_buf[18] = 0x00;

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;

	  telemetry_tx_buf[2] = id;

//	  memcpy(&telemetry_tx_buf[3], &p, 4);
//	  memcpy(&telemetry_tx_buf[7], &i, 4);
//	  memcpy(&telemetry_tx_buf[11], &d, 4);

	  *(float*)&telemetry_tx_buf[3] = p;
	  *(float*)&telemetry_tx_buf[7] = i;
	  *(float*)&telemetry_tx_buf[11] = d;

	  telemetry_tx_buf[15] = 0x00;
	  telemetry_tx_buf[16] = 0x00;
	  telemetry_tx_buf[17] = 0x00;
	  telemetry_tx_buf[18] = 0x00;

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

void pidConfig_Init(void)
{
	  if(EP_PIDGain_Read(0, &roll.in.kp, &roll.in.ki, &roll.in.kd) != 0 ||
		  EP_PIDGain_Read(1, &roll.out.kp, &roll.out.ki, &roll.out.kd) != 0 ||
		  EP_PIDGain_Read(2, &pitch.in.kp, &pitch.in.ki, &pitch.in.kd) != 0 ||
		  EP_PIDGain_Read(3, &pitch.out.kp, &pitch.out.ki, &pitch.out.kd) != 0 ||
		  EP_PIDGain_Read(4, &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd) != 0 ||
		  EP_PIDGain_Read(5, &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd) != 0)
	  {
		  printf("\nCouldn't load PID gain.\n");
	  }
	  else
	  {
//		  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
//		  uartWrite(0, &telemetry_tx_buf[0], 20);
//		  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
//		  uartWrite(0, &telemetry_tx_buf[0], 20);
//		  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
//		  uartWrite(0, &telemetry_tx_buf[0], 20);
//		  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
//		  uartWrite(0, &telemetry_tx_buf[0], 20);
//		  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
//		  uartWrite(0, &telemetry_tx_buf[0], 20);
//		  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
//		  uartWrite(0, &telemetry_tx_buf[0], 20);
		  printf("\nAll gains OK!\n\n");
	  }
}
