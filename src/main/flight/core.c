/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "rx/rx.h"
#include "rx/rc.h"

#include "rx/rc_controls.h"
#include "rx/rc_modes.h"
#include "flight/runtime_config.h"
#include "flight/stats.h"
#include "flight/core.h"

#if defined(USE_DYN_NOTCH_FILTER)
#include "dyn_notch_filter.h"
#endif

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"

//#include "drivers/gps/gps.h"

#include "accgyro/bmi270.h"
#include "barometer/barometer_dps310.h"
#include "adc/battery.h"
#include "compass/compass_qmc5883l.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_CRASHFLIP = 2,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
};

#define GYRO_WATCHDOG_DELAY 80 //  delay for gyro sync

#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

static int lastArmingDisabledReason = 0;
static timeUs_t lastDisarmTimeUs;
static int tryingToArm = ARMING_DELAYED_DISARMED;


throttleCorrectionConfig_t throttleCorrectionConfig;

void throttleCorrectionConfig_Init(void)
{
	throttleCorrectionConfig.throttle_correction_value = 0;      	// could 10 with althold or 40 for fpv
	throttleCorrectionConfig.throttle_correction_angle = 800;       // could be 80.0 deg with atlhold or 45.0 for fpv
}

void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

void disarm(flightLogDisarmReason_e reason)
{
 if (ARMING_FLAG(ARMED)) {

	 ENABLE_ARMING_FLAG(WAS_EVER_ARMED);

	 DISABLE_ARMING_FLAG(ARMED);
	 lastDisarmTimeUs = micros();

#ifdef USE_OSD
	 if (IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) || isLaunchControlActive()) {
			 osdSuppressStats(true);
	 }
#endif

#ifdef USE_BLACKBOX
	 flightLogEvent_disarm_t eventData;
	 eventData.reason = reason;
	 blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, (flightLogEventData_t*)&eventData);

	 if (blackboxConfig()->device && blackboxConfig()->mode != BLACKBOX_MODE_ALWAYS_ON) { // Close the log upon disarm except when logging mode is ALWAYS ON
			 blackboxFinish();
	 }
#else
	 UNUSED(reason);
#endif
		 //BEEP_OFF;
#ifdef USE_DSHOT
	 if (isMotorProtocolDshot() && flipOverAfterCrashActive && !featureIsEnabled(FEATURE_3D)) {
			 dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
	 }
#endif
#ifdef USE_PERSISTENT_STATS
	statsOnDisarm();
#endif
 }
}

void tryArm(void)
{
 if (armingConfig.gyro_cal_on_first_arm) {
	 //gyroStartCalibration(true);
 }

 //updateArmingStatus();

 if (!isArmingDisabled()) {
		 if (ARMING_FLAG(ARMED)) {
				 return;
		 }

		 const timeUs_t currentTimeUs = micros();

#ifdef USE_DSHOT
		 if (currentTimeUs - getLastDshotBeaconCommandTimeUs() < DSHOT_BEACON_GUARD_DELAY_US) {
				 if (tryingToArm == ARMING_DELAYED_DISARMED) {
						 if (IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
								 tryingToArm = ARMING_DELAYED_CRASHFLIP;
#ifdef USE_LAUNCH_CONTROL
						 } else if (canUseLaunchControl()) {
								 tryingToArm = ARMING_DELAYED_LAUNCH_CONTROL;
#endif
						 } else {
								 tryingToArm = ARMING_DELAYED_NORMAL;
						 }
				 }
				 return;
		 }

		 if (isMotorProtocolDshot() && isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH)) {
				 if (!(IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) || (tryingToArm == ARMING_DELAYED_CRASHFLIP))) {
						 flipOverAfterCrashActive = false;
						 if (!featureIsEnabled(FEATURE_3D)) {
								 dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
						 }
				 } else {
						 flipOverAfterCrashActive = true;
#ifdef USE_RUNAWAY_TAKEOFF
						 runawayTakeoffCheckDisabled = false;
#endif
						 if (!featureIsEnabled(FEATURE_3D)) {
								 dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
						 }
				 }
		 }
#endif

#ifdef USE_LAUNCH_CONTROL
		 if (!flipOverAfterCrashActive && (canUseLaunchControl() || (tryingToArm == ARMING_DELAYED_LAUNCH_CONTROL))) {
				 if (launchControlState == LAUNCH_CONTROL_DISABLED) {  // only activate if it hasn't already been triggered
						 launchControlState = LAUNCH_CONTROL_ACTIVE;
				 }
		 }
#endif

#ifdef USE_OSD
		 osdSuppressStats(false);
#endif
		 ENABLE_ARMING_FLAG(ARMED);

		 resetTryingToArm();

#ifdef USE_ACRO_TRAINER
		 pidAcroTrainerInit();
#endif // USE_ACRO_TRAINER

		 if (isModeActivationConditionPresent(BOXPREARM)) {
				 ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
		 }
		 imuQuaternionHeadfreeOffsetSet();

#if defined(USE_DYN_NOTCH_FILTER)
		 resetMaxFFT();
#endif

		 disarmAt = currentTimeUs + armingConfig.auto_disarm_delay * 1e6;   // start disarm timeout, will be extended when throttle is nonzero

		 lastArmingDisabledReason = 0;

#ifdef USE_GPS
		 GPS_reset_home_position();

		 //beep to indicate arming
		 if (featureIsEnabled(FEATURE_GPS)) {
				 if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
						 beeper(BEEPER_ARMING_GPS_FIX);
				 } else {
						 beeper(BEEPER_ARMING_GPS_NO_FIX);
				 }
		 } else {
				 beeper(BEEPER_ARMING);
		 }
#else
		 //beeper(BEEPER_ARMING);
#endif

#ifdef USE_PERSISTENT_STATS
		 statsOnArm();
#endif

	resetTryingToArm();
//		 if (!isFirstArmingGyroCalibrationRunning()) {
//				 int armingDisabledReason = ffs(getArmingDisableFlags());
//				 if (lastArmingDisabledReason != armingDisabledReason) {
//						 lastArmingDisabledReason = armingDisabledReason;
//
//						 //beeperWarningBeeps(armingDisabledReason);
//				 }
//		 }
 }
}

 // Automatic ACC Offset Calibration
 bool AccInflightCalibrationArmed = false;
 bool AccInflightCalibrationMeasurementDone = false;
 bool AccInflightCalibrationSavetoEEProm = false;
 bool AccInflightCalibrationActive = false;
 uint16_t InflightcalibratingA = 0;

 void handleInflightCalibrationStickPosition(void)
 {
     if (AccInflightCalibrationMeasurementDone) {
         // trigger saving into eeprom after landing
         AccInflightCalibrationMeasurementDone = false;
         AccInflightCalibrationSavetoEEProm = true;
     } else {
         AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
         if (AccInflightCalibrationArmed) {
             //beeper(BEEPER_ACC_CALIBRATION);
         } else {
             //beeper(BEEPER_ACC_CALIBRATION_FAIL);
         }
     }
 }

 static void updateInflightCalibrationState(void)
 {
     if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig.mincheck && !IS_RC_MODE_ACTIVE(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
         InflightcalibratingA = 50;
         AccInflightCalibrationArmed = false;
     }
     if (IS_RC_MODE_ACTIVE(BOXCALIB)) {      // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
         if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
             InflightcalibratingA = 50;
         AccInflightCalibrationActive = true;
     } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
         AccInflightCalibrationMeasurementDone = false;
         AccInflightCalibrationSavetoEEProm = true;
     }
 }

#if defined(USE_GPS) || defined(USE_MAG)
static void updateMagHold(void)
{
    if (fabsf(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -GET_DIRECTION(rcControlsConfig.yaw_control_reversed);
//        if (isUpright()) {
//            rcCommand[YAW] -= dif * 1.333 / 30;    // 18 deg
//        }
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}
#endif

#ifdef USE_VTX_CONTROL
static bool canUpdateVTX(void)
{
#ifdef USE_VTX_RTC6705
    return vtxRTC6705CanUpdate();
#endif
    return true;
}
#endif

#if defined(USE_RUNAWAY_TAKEOFF) || defined(USE_GPS_RESCUE)
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        const uint8_t deadband = axis == FD_YAW ? rcControlsConfig()->yaw_deadband : rcControlsConfig()->deadband;
        uint8_t stickPercent = 0;
        if ((rcData[axis] >= PWM_RANGE_MAX) || (rcData[axis] <= PWM_RANGE_MIN)) {
            stickPercent = 100;
        } else {
            if (rcData[axis] > (rxConfig()->midrc + deadband)) {
                stickPercent = ((rcData[axis] - rxConfig()->midrc - deadband) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - deadband);
            } else if (rcData[axis] < (rxConfig()->midrc - deadband)) {
                stickPercent = ((rxConfig()->midrc - deadband - rcData[axis]) * 100) / (rxConfig()->midrc - deadband - PWM_RANGE_MIN);
            }
        }
        if (stickPercent >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}
#endif

#ifdef USE_RUNAWAY_TAKEOFF
// allow temporarily disabling runaway takeoff prevention if we are connected
// to the configurator and the ARMING_DISABLED_MSP flag is cleared.
void runawayTakeoffTemporaryDisable(uint8_t disableFlag)
{
    runawayTakeoffTemporarilyDisabled = disableFlag;
}
#endif


/*
 * processRx called from taskUpdateRxMain
 */
bool processRx(timeUs_t currentTimeUs)
{
    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

    //updateRcRefreshRate(currentTimeUs);

    //updateRSSI(currentTimeUs);

    return true;
}

void processRxModes(uint32_t currentTimeUs)
{
    static bool armedBeeperOn = false;
#ifdef USE_TELEMETRY
    static bool sharedPortTelemetryEnabled = false;
#endif
    const throttleStatus_e throttleStatus = calculateThrottleStatus();

     //When armed and motors aren't spinning, do beeps and then disarm
     //board after delay so users without buzzer won't lose fingers.
     //mixTable constrains motor commands, so checking  throttleStatus is enough
     const timeUs_t autoDisarmDelayUs = armingConfig.auto_disarm_delay * 1e6;

     disarmAt = currentTimeUs + autoDisarmDelayUs;  // extend auto-disarm timer

     if (!(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))
#ifdef USE_CMS
        && !cmsInMenu
#endif
        ) {
        processRcStickPositions();
    }


	 //updateInflightCalibrationState();

    updateActivatedModes();

     bool canUseHorizonMode = true;

     if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
         // bumpless transfer to Level mode
         canUseHorizonMode = false;

         if (!FLIGHT_MODE(ANGLE_MODE)) {
             ENABLE_FLIGHT_MODE(ANGLE_MODE);
         }
     } else {
         DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
     }

     if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

         DISABLE_FLIGHT_MODE(ANGLE_MODE);

         if (!FLIGHT_MODE(HORIZON_MODE)) {
             ENABLE_FLIGHT_MODE(HORIZON_MODE);
         }
     } else {
         DISABLE_FLIGHT_MODE(HORIZON_MODE);
     }

#ifdef USE_GPS_RESCUE
    if (ARMING_FLAG(ARMED) && (IS_RC_MODE_ACTIVE(BOXGPSRESCUE) || (failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE))) {
        if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    }
#endif

	if (!IS_RC_MODE_ACTIVE(BOXPREARM) && ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
	 DISABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
	}

#if defined(USE_ACC) || defined(USE_MAG)
#if defined(USE_GPS) || defined(USE_MAG)
	 if (IS_RC_MODE_ACTIVE(BOXMAG)) {
		 if (!FLIGHT_MODE(MAG_MODE)) {
			 ENABLE_FLIGHT_MODE(MAG_MODE);
			 magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
		 }
	 } else {
		 DISABLE_FLIGHT_MODE(MAG_MODE);
	 }
#endif
	 if (IS_RC_MODE_ACTIVE(BOXHEADFREE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
		 if (!FLIGHT_MODE(HEADFREE_MODE)) {
			 ENABLE_FLIGHT_MODE(HEADFREE_MODE);
		 }
	 } else {
		 DISABLE_FLIGHT_MODE(HEADFREE_MODE);
	 }
	 if (IS_RC_MODE_ACTIVE(BOXHEADADJ) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
		 if (imuQuaternionHeadfreeOffsetSet()) {
			//beeper(BEEPER_RX_SET);
		 }
	 }
#endif
#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        bool enableSharedPortTelemetry = (!isModeActivationConditionPresent(BOXTELEMETRY) && ARMING_FLAG(ARMED)) || (isModeActivationConditionPresent(BOXTELEMETRY) && IS_RC_MODE_ACTIVE(BOXTELEMETRY));
        if (enableSharedPortTelemetry && !sharedPortTelemetryEnabled) {
            //mspSerialReleaseSharedTelemetryPorts();
            telemetryCheckState();

            sharedPortTelemetryEnabled = true;
        } else if (!enableSharedPortTelemetry && sharedPortTelemetryEnabled) {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            //mspSerialAllocatePorts();

            sharedPortTelemetryEnabled = false;
        }
    }
#endif
}

static void RcCommand(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= rxConfig.mincheck)
    {
        resetYawAxis();
    }

    processRcCommand();
}

// Function for loop trigger
void MainPidLoop(timeUs_t currentTimeUs)
{
    RcCommand(currentTimeUs);
    pidUpdate(currentTimeUs);
	mixTable(currentTimeUs);
	writeMotors();

#if defined(USE_GPS) || defined(USE_MAG)
        updateMagHold();
#endif
}

uint32_t getLastDisarmTimeUs(void)
{
    return lastDisarmTimeUs;
}

bool isTryingToArm()
{
    return (tryingToArm != ARMING_DELAYED_DISARMED);
}

void resetTryingToArm()
{
    tryingToArm = ARMING_DELAYED_DISARMED;
}
