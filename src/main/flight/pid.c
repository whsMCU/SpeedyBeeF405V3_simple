/*
 * pid.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */
#include "accgyro/bmi270.h"
#include "adc/battery.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/runtime_config.h"
#include "flight/mixer.h"
#include "flight/core.h"
#include "rx/rx.h"
#include "rx/rc_controls.h"
#include "pg/parameter.h"


PIDDouble roll;
PIDDouble pitch;
PIDSingle yaw_heading;
PIDSingle yaw_rate;

pidAxisData_t pidData[XYZ_AXIS_COUNT];

unsigned short ccr1, ccr2, ccr3, ccr4;

float yaw_heading_reference;

#define DT 0.000312f
#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
#define ITERM_RELAX_SETPOINT_THRESHOLD 40.0f
#define ITERM_RELAX_CUTOFF_DEFAULT 15

pidProfile_t pidProfile;

void pidProfile_Init(void)
{
	roll.in.kp = 0.5;
	roll.in.ki = 0.5;
	roll.in.kd = 0.17;
	roll.out.kp = 4.5;
	roll.out.ki = 0.3;
	roll.out.kd = 0.4;

	pitch.in.kp = 0.65;
	pitch.in.ki = 0.5;
	pitch.in.kd = 0.15;
	pitch.out.kp = 4.5;
	pitch.out.ki = 0.3;
	pitch.out.kd = 0.4;

	yaw_heading.kp = 5;
	yaw_heading.ki = 0;
	yaw_heading.kd = 2;

	yaw_rate.kp = 1.5;
	yaw_rate.ki = 0;
	yaw_rate.kd = 0.2;

	pidProfile.pid[PID_ROLL].P = 45;
	pidProfile.pid[PID_ROLL].I = 80;
	pidProfile.pid[PID_ROLL].D = 40;
	pidProfile.pid[PID_ROLL].F = 120;

	pidProfile.pid[PID_PITCH].P = 47;
	pidProfile.pid[PID_PITCH].I = 84;
	pidProfile.pid[PID_PITCH].D = 46;
	pidProfile.pid[PID_PITCH].F = 125;

	pidProfile.pid[PID_YAW].P = 45;
	pidProfile.pid[PID_YAW].I = 80;
	pidProfile.pid[PID_YAW].D = 0;
	pidProfile.pid[PID_YAW].F = 120;

	pidProfile.pid[PID_LEVEL].P = 50;
	pidProfile.pid[PID_LEVEL].I = 50;
	pidProfile.pid[PID_LEVEL].D = 75;
	pidProfile.pid[PID_LEVEL].F = 0;

	pidProfile.pid[PID_MAG].P = 40;
	pidProfile.pid[PID_MAG].I = 0;
	pidProfile.pid[PID_MAG].D = 0;
	pidProfile.pid[PID_MAG].F = 0;

	pidProfile.pidSumLimit = PIDSUM_LIMIT;
	pidProfile.pidSumLimitYaw = PIDSUM_LIMIT_YAW;
	pidProfile.yaw_lowpass_hz = 100;
	pidProfile.dterm_notch_hz = 0;
	pidProfile.dterm_notch_cutoff = 0;
	pidProfile.itermWindupPointPercent = 85;
	pidProfile.pidAtMinThrottle = PID_STABILISATION_ON;
	pidProfile.levelAngleLimit = 55;
	pidProfile.yawRateAccelLimit = 0;
	pidProfile.rateAccelLimit = 0;
	pidProfile.itermThrottleThreshold = 250;
	pidProfile.itermAcceleratorGain = 3500;
	pidProfile.crash_time = 500;          // ms
	pidProfile.crash_delay = 0;           // ms
	pidProfile.crash_recovery_angle = 10; // degrees
	pidProfile.crash_recovery_rate = 100; // degrees/second
	pidProfile.crash_dthreshold = 50;     // degrees/second/second
	pidProfile.crash_gthreshold = 400;    // degrees/second
	pidProfile.crash_setpoint_threshold = 350; // degrees/second
	pidProfile.crash_recovery = PID_CRASH_RECOVERY_OFF; // off by default
	pidProfile.horizon_tilt_effect = 75;
	pidProfile.horizon_tilt_expert_mode = false;
	pidProfile.crash_limit_yaw = 200;
	pidProfile.itermLimit = 400;
	pidProfile.throttle_boost = 5;
	pidProfile.throttle_boost_cutoff = 15;
	pidProfile.iterm_rotation = false;
	pidProfile.iterm_relax = ITERM_RELAX_RP;
	pidProfile.iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT;
	pidProfile.iterm_relax_type = ITERM_RELAX_SETPOINT;
	pidProfile.acro_trainer_angle_limit = 20;
	pidProfile.acro_trainer_lookahead_ms = 50;
	pidProfile.acro_trainer_debug_axis = FD_ROLL;
	pidProfile.acro_trainer_gain = 75;
	pidProfile.abs_control_gain = 0;
	pidProfile.abs_control_limit = 90;
	pidProfile.abs_control_error_limit = 20;
	pidProfile.abs_control_cutoff = 11;
	pidProfile.antiGravityMode = ANTI_GRAVITY_SMOOTH;
	pidProfile.dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT;
	 // NOTE: dynamic lpf is enabled by default so this setting is actually
	 // overridden and the static lowpass 1 is disabled. We can't set this
	 // value to 0 otherwise Configurator versions 10.4 and earlier will also
	 // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
	pidProfile.dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT;   // second Dterm LPF ON by default
	pidProfile.dterm_lpf1_type = FILTER_PT1;
	pidProfile.dterm_lpf2_type = FILTER_PT1;
	pidProfile.dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT;
	pidProfile.dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT;
	pidProfile.launchControlMode = LAUNCH_CONTROL_MODE_NORMAL;
	pidProfile.launchControlThrottlePercent = 20;
	pidProfile.launchControlAngleLimit = 0;
	pidProfile.launchControlGain = 40;
	pidProfile.launchControlAllowTriggerReset = true;
	pidProfile.use_integrated_yaw = false;
	pidProfile.integrated_yaw_relax = 200;
	pidProfile.thrustLinearization = 0;

	pidProfile.d_min[X] = 30;
	pidProfile.d_min[Y] = 34;
	pidProfile.d_min[Z] = 0;

	pidProfile.d_min_gain = 37;
	pidProfile.d_min_advance = 20;
	pidProfile.motor_output_limit = 100;
	pidProfile.auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY;
	pidProfile.transient_throttle_limit = 0;

	pidProfile.dterm_lpf1_dyn_expo = 5;
	pidProfile.level_race_mode = false;
	pidProfile.vbat_sag_compensation = 0;
}

void pidResetIterm(void)
{
	for (int axis = 0; axis < 3; axis++) {
	 pidData[axis].I = 0.0f;
	#if defined(USE_ABSOLUTE_CONTROL)
	 axisError[axis] = 0.0f;
	#endif
 }
}

//void pidUpdateTpaFactor(float throttle)
//{
//    const float tpaBreakpoint = (currentControlRateProfile->tpa_breakpoint - 1000) / 1000.0f;
//    float tpaRate = currentControlRateProfile->tpa_rate / 100.0f;
//    if (throttle > tpaBreakpoint) {
//        if (throttle < 1.0f) {
//            tpaRate *= (throttle - tpaBreakpoint) / (1.0f - tpaBreakpoint);
//        }
//    } else {
//        tpaRate = 0.0f;
//    }
//    pidRuntime.tpaFactor = 1.0f - tpaRate;
//}
//
//void pidUpdateAntiGravityThrottleFilter(float throttle)
//{
//    if (pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) {
//        // calculate a boost factor for P in the same way as for I when throttle changes quickly
//        const float antiGravityThrottleLpf = pt1FilterApply(&pidRuntime.antiGravityThrottleLpf, throttle);
//        // focus P boost on low throttle range only
//        if (throttle < 0.5f) {
//            pidRuntime.antiGravityPBoost = 0.5f - throttle;
//        } else {
//            pidRuntime.antiGravityPBoost = 0.0f;
//        }
//        // use lowpass to identify start of a throttle up, use this to reduce boost at start by half
//        if (antiGravityThrottleLpf < throttle) {
//            pidRuntime.antiGravityPBoost *= 0.5f;
//        }
//        // high-passed throttle focuses boost on faster throttle changes
//        pidRuntime.antiGravityThrottleHpf = fabsf(throttle - antiGravityThrottleLpf);
//        pidRuntime.antiGravityPBoost = pidRuntime.antiGravityPBoost * pidRuntime.antiGravityThrottleHpf;
//        // smooth the P boost at 3hz to remove the jagged edges and prolong the effect after throttle stops
//        pidRuntime.antiGravityPBoost = pt1FilterApply(&pidRuntime.antiGravitySmoothLpf, pidRuntime.antiGravityPBoost);
//    }
//}


void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
	axis->out.reference = set_point_angle;	//Set point of outer PID control
	axis->out.meas_value = angle;			//BNO080 rotation angle

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop
	axis->out.p_result = axis->out.error * axis->out.kp;			//Calculate P result of outer loop

	axis->out.error_sum = axis->out.error_sum + axis->out.error * DT;	//Define summation of outer loop
#define OUT_ERR_SUM_MAX 500
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
	else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Calculate I result of outer loop

	axis->out.error_deriv = -rate;										//Define derivative of outer loop (rate = ICM-20602 Angular Rate)

#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Calculate D result of inner loop
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop
	/****************************************************************************************/

	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = rate;					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;	//Define summation of inner loop
#define IN_ERR_SUM_MAX 500
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->out.error_sum > IN_ERR_SUM_MAX) axis->out.error_sum = IN_ERR_SUM_MAX;
	else if(axis->out.error_sum < IN_I_ERR_MIN) axis->out.error_sum = IN_I_ERR_MIN;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Calculate I result of inner loop

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Define derivative of inner loop
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
}

void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point_angle, float angle/*Rotation Angle*/, float rate/*Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Position) *************/
	axis->reference = set_point_angle;	//Set point of yaw heading @ yaw stick is center.
	axis->meas_value = angle;			//Current BNO080_Yaw angle @ yaw stick is center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw angle control

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;

	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw angle control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw angle control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw angle control

	axis->error_deriv = -rate;						//Define differentiation of yaw angle control
	axis->d_result = axis->error_deriv * axis->kd;	//Calculate D result of yaw angle control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw angle control
	/***************************************************************/
}

void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float rate/*Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_rate;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = rate;			//Current ICM20602.gyro_z @ yaw stick is not center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw rate control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;	//Define differentiation of yaw rate control
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);
}

void pidUpdate(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);

	Double_Roll_Pitch_PID_Calculation(&roll, rcCommand[ROLL] * 0.1f, (attitude.values.roll/10), mpu.gyro.gyroADC[X]);
    Double_Roll_Pitch_PID_Calculation(&pitch, rcCommand[PITCH] * 0.1f, (attitude.values.pitch/10), mpu.gyro.gyroADC[Y]);

	if(rcData[THROTTLE] < rxConfig.mincheck || !ARMING_FLAG(ARMED))
	{
	  Reset_All_PID_Integrator();
	}

	if(rcData[YAW] < 1485 || rcData[YAW] > 1515)
	{
	  yaw_heading_reference = (attitude.values.yaw/10);
	  Single_Yaw_Rate_PID_Calculation(&yaw_rate, rcCommand[YAW], mpu.gyro.gyroADC[Z]);
	  pidData[FD_ROLL].Sum = roll.in.pid_result;
	  pidData[FD_PITCH].Sum = pitch.in.pid_result;
	  pidData[FD_YAW].Sum = yaw_rate.pid_result;
//	  motor[0] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
//	  motor[1] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
//	  motor[2] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
//	  motor[3] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
	}
	else
	{
	  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, (attitude.values.yaw/10), mpu.gyro.gyroADC[Z]);
	  pidData[FD_ROLL].Sum = roll.in.pid_result;
	  pidData[FD_PITCH].Sum = pitch.in.pid_result;
	  pidData[FD_YAW].Sum = yaw_heading.pid_result;
//	  motor[0] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result;
//	  motor[1] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
//	  motor[2] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result;
//	  motor[3] = 1000 + (rcData[THROTTLE] - 1000) * 0.5 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
	}
}
