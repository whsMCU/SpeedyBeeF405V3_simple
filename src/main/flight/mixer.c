/*
 * mixer.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "flight/mixer.h"
#include "flight/pid.h"
#include "rx/rc_controls.h"
#include "common/maths.h"


uint32_t time_manual_motor;
uint8_t Manual_Motor_flag = 0;

int16_t motor[4];
int16_t M_motor[4];

#define pulseScale 1.679999
#define pulseOffset -1260

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

void mixTable(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	uint8_t i = 0;
	if (rcCommand[THROTTLE] > 4000) rcCommand[THROTTLE] = 4000;                                   //We need some room to keep full control at full throttle.
	for (i = 0; i < 4; i++){
		motor[i] = (rcCommand[THROTTLE] * (int16_t)currentMixer[i].THROTTLE) + ((int16_t)roll.in.pid_result * (int16_t)currentMixer[i].ROLL) + ((int16_t)pitch.in.pid_result * (int16_t)currentMixer[i].PITCH) + ((1 * (int16_t)yaw_rate.pid_result) * (int16_t)currentMixer[i].YAW);

		//motor[i] = lrintf((motor[i] * pulseScale) + pulseOffset);
		//constrain(motor[i], 2250, 4500);
	}

    if(Manual_Motor_flag == true){
		TIM4->CCR1 = motor[0];  // Actual : REAR_L
		TIM4->CCR2 = motor[1];  // Actual : FRONT_R
		TIM4->CCR3 = motor[2];  // Actual : FRONT_L
		TIM4->CCR4 = motor[3];  // Actual : REAR_R
    }else{
		TIM4->CCR1 = motor[0];  // Actual : REAR_L
		TIM4->CCR2 = motor[1];  // Actual : FRONT_R
		TIM4->CCR3 = motor[2];  // Actual : FRONT_L
		TIM4->CCR4 = motor[3];  // Actual : REAR_R
  }
}

