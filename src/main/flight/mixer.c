/*
 * mixer.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "flight/mixer.h"


uint32_t time_manual_motor;
uint8_t Manual_Motor_flag = 0;

int16_t motor[4];
int16_t M_motor[4];

// Custom mixer data per motor
typedef struct motorMixer_t {
    float THROTTLE;
    float ROLL;
    float PITCH;
    float YAW;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

static motorMixer_t currentMixer[4];

static const motorMixer_t mixerQuadP[] = {
//	{ 1.0f,  0.0f, -1.0f,  1.0f },          // REAR (CW)   1
//  { 1.0f, -1.0f,  0.0f, -1.0f },        // RIGHT (CCW)  2
//  { 1.0f,  1.0f,  0.0f, -1.0f },        // LEFT  (CCW)  3
//	{ 1.0f,  0.0f,  1.0f,  1.0f },          // FRONT  (CW)   4

  { 1.0f,  0.0f,  1.0f,  0.0f },          // REAR (CW)   1  //2, 0.2. 0.1
  { 1.0f,  0.0f, -1.0f,  0.0f },        // RIGHT (CCW)  2
  { 0.0f,  0.0f,  0.0f,  0.0f },        // LEFT  (CCW)  3
  { 0.0f,  0.0f,  0.0f,  0.0f },          // FRONT  (CW)   4
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f,  1.0f,  1.0f, -1.0f },          // (CW)  0  //REAR_L
    { 1.0f, -1.0f, -1.0f, -1.0f },          // (CW)  1  //FRONT_R
    { 1.0f,  1.0f, -1.0f,  1.0f },          // (CCW) 2  //FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // (CCW) 3  //REAR_R
}; // THR,  ROLL,   PITCH,   YAW

const mixer_t mixers[] = {
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
};

void mixerInit(void)
{
    for (int i = 0; i < 4; i++)
	{
    	currentMixer[i] = mixers[QuadX].motor[i];   //0 = QuadP, 1 = QuadX
	}
}

void mixTable(void)
{
//	uint8_t i = 0;
//			if (RC.rcCommand[THROTTLE] > 4000) RC.rcCommand[THROTTLE] = 4000;                                   //We need some room to keep full control at full throttle.
//			for (i = 0; i < 4; i++){
//				motor[i] = (RC.rcCommand[THROTTLE] * (int16_t)currentMixer[i].THROTTLE) + ((int16_t)pid.output2[ROLL] * (int16_t)currentMixer[i].ROLL) + ((int16_t)pid.output2[PITCH] * (int16_t)currentMixer[i].PITCH) + ((1 * (int16_t)pid.output2[YAW]) * (int16_t)currentMixer[i].YAW);

  #ifdef VOLTAGEDROP_COMPENSATION
    #if (VBATNOMINAL == 42) // 1S
      #define GOV_R_NUM 12
			static int8_t g[] = { 0,4,10,17,25,34,44,55,67,80,94,109,126 };
    #elif (VBATNOMINAL == 126) // 3S
      #define GOV_R_NUM 36
			static int8_t g[] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
    #endif
		uint8_t v = constrain( VBATNOMINAL - constrain(BAT.VBAT, VBATLEVEL_CRIT, VBATNOMINAL), 0, GOV_R_NUM);
		for (i = 0; i < 4; i++) {
      #ifdef MOTOR_DC
        motor[i] += ( ( (int32_t)(motor[i]) * (int32_t)g[v] ) )/ 500;
      #elif MOTOR_ESC
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
      #endif
		}
  #endif


//	motor[i] = constrain(motor[i], 2250, 4500);
//
//
//
//  if(RC.rcCommand[THROTTLE] < 2350 || f.ARMED == 0){
//    motor[i] = 2250;

//		pid.output1[i] = 0;
//		pid.output2[i] = 0;
//		pid.Iterm[i] = 0;
//		pid.Iterm1[i] = 0;
//		pid.Iterm2[i] = 0;
//	}
//  if(micros() - time_manual_motor >= 500000){
//    Manual_Motor_flag = false;
//  }
//  if(Manual_Motor_flag == true){
//    TIM4->CCR1 = M_motor[0];  // Actual : REAR_L
//    TIM4->CCR2 = M_motor[1];  // Actual : FRONT_R
//    TIM4->CCR3 = M_motor[2];  // Actual : FRONT_L
//    TIM4->CCR4 = M_motor[3];  // Actual : REAR_R
//  }else{
//    TIM4->CCR1 = motor[0];  // Actual : REAR_L
//    TIM4->CCR2 = motor[1];  // Actual : FRONT_R
//    TIM4->CCR3 = motor[2];  // Actual : FRONT_L
//    TIM4->CCR4 = motor[3];  // Actual : REAR_R
//  }
//
//  }
//}
}
