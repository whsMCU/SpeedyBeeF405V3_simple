/*
 * pid.h
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_PID_H_
#define SRC_COMMON_CORE_PID_H_

#include "common/time.h"
#include "common/axis.h"
#include "hw.h"

#define PID_CONTROLLER_BETAFLIGHT   1
#define PID_MIXER_SCALING           1000.0f
#define PID_SERVO_MIXER_SCALING     0.7f
#define PIDSUM_LIMIT                500
#define PIDSUM_LIMIT_YAW            400
#define PIDSUM_LIMIT_MIN            100
#define PIDSUM_LIMIT_MAX            1000

#define DTERM_LPF1_DYN_MIN_HZ_DEFAULT 75
#define DTERM_LPF1_DYN_MAX_HZ_DEFAULT 150
#define DTERM_LPF2_HZ_DEFAULT 150

#define SIMPLIFIED_TUNING_PIDS_MIN 0
#define SIMPLIFIED_TUNING_FILTERS_MIN 10
#define SIMPLIFIED_TUNING_MAX 200
#define SIMPLIFIED_TUNING_DEFAULT 100
#define SIMPLIFIED_TUNING_D_DEFAULT 100

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    PID_STABILISATION_OFF = 0,
    PID_STABILISATION_ON
} pidStabilisationState_e;

typedef enum {
    PID_CRASH_RECOVERY_OFF = 0,
    PID_CRASH_RECOVERY_ON,
    PID_CRASH_RECOVERY_BEEP,
    PID_CRASH_RECOVERY_DISARM,
} pidCrashRecovery_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
} pidf_t;

typedef enum {
    ANTI_GRAVITY_SMOOTH,
    ANTI_GRAVITY_STEP
} antiGravityMode_e;

typedef enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
    ITERM_RELAX_RP_INC,
    ITERM_RELAX_RPY_INC,
    ITERM_RELAX_COUNT,
} itermRelax_e;

typedef enum {
    ITERM_RELAX_GYRO,
    ITERM_RELAX_SETPOINT,
    ITERM_RELAX_TYPE_COUNT,
} itermRelaxType_e;

typedef struct pidProfile_s {
    uint16_t yaw_lowpass_hz;                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_lpf1_static_hz;          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff

    pidf_t  pid[PID_ITEM_COUNT];

    uint8_t dterm_lpf1_type;                // Filter type for dterm lowpass 1
    uint8_t itermWindupPointPercent;        // iterm windup threshold, percent motor saturation
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t levelAngleLimit;                // Max angle in degrees in level mode

    uint8_t horizon_tilt_effect;            // inclination factor for Horizon mode
    uint8_t horizon_tilt_expert_mode;       // OFF or ON

    // Betaflight PID controller parameters
    uint8_t  antiGravityMode;             // type of anti gravity method
    uint16_t itermThrottleThreshold;        // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain;          // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t crash_dthreshold;              // dterm crash value
    uint16_t crash_gthreshold;              // gyro crash value
    uint16_t crash_setpoint_threshold;      // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    uint16_t crash_time;                    // ms
    uint16_t crash_delay;                   // ms
    uint8_t crash_recovery_angle;           // degrees
    uint8_t crash_recovery_rate;            // degree/second
    uint16_t crash_limit_yaw;               // limits yaw errorRate, so crashes don't cause huge throttle increase
    uint16_t itermLimit;
    uint16_t dterm_lpf2_static_hz;          // Static Dterm lowpass 2 filter cutoff value in hz
    uint8_t crash_recovery;                 // off, on, on and beeps when it is in crash recovery mode
    uint8_t throttle_boost;                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    uint8_t throttle_boost_cutoff;          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    uint8_t iterm_rotation;                 // rotates iterm to translate world errors to local coordinate system
    uint8_t iterm_relax_type;               // Specifies type of relax algorithm
    uint8_t iterm_relax_cutoff;             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax;                    // Enable iterm suppression during stick input
    uint8_t acro_trainer_angle_limit;       // Acro trainer roll/pitch angle limit in degrees
    uint8_t acro_trainer_debug_axis;        // The axis for which record debugging values are captured 0=roll, 1=pitch
    uint8_t acro_trainer_gain;              // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint16_t acro_trainer_lookahead_ms;     // The lookahead window in milliseconds used to reduce overshoot
    uint8_t abs_control_gain;               // How strongly should the absolute accumulated error be corrected for
    uint8_t abs_control_limit;              // Limit to the correction
    uint8_t abs_control_error_limit;        // Limit to the accumulated error
    uint8_t abs_control_cutoff;             // Cutoff frequency for path estimation in abs control
    uint8_t dterm_lpf2_type;                // Filter type for 2nd dterm lowpass
    uint16_t dterm_lpf1_dyn_min_hz;         // Dterm lowpass filter 1 min hz when in dynamic mode
    uint16_t dterm_lpf1_dyn_max_hz;         // Dterm lowpass filter 1 max hz when in dynamic mode
    uint8_t launchControlMode;              // Whether launch control is limited to pitch only (launch stand or top-mount) or all axes (on battery)
    uint8_t launchControlThrottlePercent;   // Throttle percentage to trigger launch for launch control
    uint8_t launchControlAngleLimit;        // Optional launch control angle limit (requires ACC)
    uint8_t launchControlGain;              // Iterm gain used while launch control is active
    uint8_t launchControlAllowTriggerReset; // Controls trigger behavior and whether the trigger can be reset
    uint8_t use_integrated_yaw;             // Selects whether the yaw pidsum should integrated
    uint8_t integrated_yaw_relax;           // Specifies how much integrated yaw should be reduced to offset the drag based yaw component
    uint8_t thrustLinearization;            // Compensation factor for pid linearization
    uint8_t d_min[XYZ_AXIS_COUNT];          // Minimum D value on each axis
    uint8_t d_min_gain;                     // Gain factor for amount of gyro / setpoint activity required to boost D
    uint8_t d_min_advance;                  // Percentage multiplier for setpoint input to boost algorithm
    uint8_t motor_output_limit;             // Upper limit of the motor output (percent)
    int8_t auto_profile_cell_count;         // Cell count for this profile to be used with if auto PID profile switching is used
    uint8_t transient_throttle_limit;       // Maximum DC component of throttle change to mix into throttle to prevent airmode mirroring noise

    uint8_t dterm_lpf1_dyn_expo;            // set the curve for dynamic dterm lowpass filter
    uint8_t level_race_mode;                // NFE race mode - when true pitch setpoint calculation is gyro based in level mode
    uint8_t vbat_sag_compensation;          // Reduce motor output by this percentage of the maximum compensation amount

} pidProfile_t;

extern pidProfile_t pidProfile;

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;

    float Sum;
} pidAxisData_t;

typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;

	float reference;
	float meas_value;
	float meas_value_prev;
	float error;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;

	float p_result;
	float i_result;
	float d_result;

	float pid_result;

}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;

extern pidAxisData_t pidData[XYZ_AXIS_COUNT];

extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;

void pidProfile_Init(void);
void pidResetIterm(void);
void pidUpdateTpaFactor(float throttle);
void pidUpdateAntiGravityThrottleFilter(float throttle);

void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle, float rate);
void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point, float value);
void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point, float angle, float rate);
void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);
void pidUpdate(timeUs_t currentTimeUs);

#endif /* SRC_COMMON_CORE_PID_H_ */
