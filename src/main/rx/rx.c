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
#include <stdlib.h>

#include <string.h>

#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"
#include "common/axis.h"
#include "common/filter.h"

#include "flight/runtime_config.h"
#include "flight/imu.h"
#include "flight/stats.h"
#include "flight/core.h"

#include "rx/rc_controls.h"
#include "rx/rc_modes.h"
#include "rx/rx.h"
#include "rx/crsf.h"

const char rcChannelLetters[] = "AERT12345678abcdefgh";

static uint16_t rssi = 0;                  // range: [0;1023]
static int16_t rssiDbm = CRSF_RSSI_MIN;    // range: [-130,20]
static uint32_t lastMspRssiUpdateUs = 0;

static pt1Filter_t frameErrFilter;

#ifdef USE_RX_LINK_QUALITY_INFO
static uint16_t linkQuality = 0;
static uint8_t rfMode = 0;
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
static uint16_t uplinkTxPwrMw = 0;  //Uplink Tx power in mW
#endif

#define RSSI_ADC_DIVISOR (4096 / 1024)
#define RSSI_OFFSET_SCALING (1024 / 100.0f)

rssiSource_e rssiSource;
linkQualitySource_e linkQualitySource;

static bool rxDataProcessingRequired = false;
//static bool auxiliaryProcessingRequired = false;

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;
static uint8_t rxChannelCount;

static uint32_t needRxSignalBefore = 0;
//static uint32_t suspendRxSignalUntil = 0;
//static uint8_t  skipRxSamples = 0;

float rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // last received raw value, as it comes
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];           // scaled, modified, checked and constrained values
uint32_t validRxSignalTimeout[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULSE_TIME_MS 300                   // hold time in milliseconds after bad channel or Rx link loss
// will not be actioned until the nearest multiple of 100ms
#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_20_MS (20 * 1000)                         // 20ms in us
#define DELAY_100_MS (100 * 1000)                       // 100ms in us
#define DELAY_1500_MS (1500 * 1000)                     // 1.5 seconds in us
#define SKIP_RC_SAMPLES_ON_RESUME  2                    // flush 2 samples to drop wrong measurements (timing independent)

#define DEFAULT_AUX_CHANNEL_COUNT       6

rxRuntimeState_t rxRuntimeState;
static uint8_t rcSampleIndex = 0;

rxConfig_t rxConfig;

void rxConfig_Init(void)
{
	rxConfig.halfDuplex = 0;
	rxConfig.serialrx_provider = SERIALRX_CRSF;
	rxConfig.serialrx_inverted = 0;
	rxConfig.spektrum_sat_bind = 0;
	rxConfig.spektrum_sat_bind_autoreset = 1;
	rxConfig.midrc = RX_MID_USEC;
	rxConfig.mincheck = 1050;
	rxConfig.maxcheck = 1900;
	rxConfig.rx_min_usec = RX_MIN_USEC;          // any of first 4 channels below this value will trigger rx loss detection
	rxConfig.rx_max_usec = RX_MAX_USEC;         // any of first 4 channels above this value will trigger rx loss detection
	rxConfig.rssi_src_frame_errors = false;
	rxConfig.rssi_channel = 0;
	rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
	rxConfig.rssi_offset = 0;
	rxConfig.rssi_invert = 0;
	rxConfig.rssi_src_frame_lpf_period = 30;
	rxConfig.fpvCamAngleDegrees = 0;
	rxConfig.airModeActivateThreshold = 25;
	rxConfig.max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT;
	rxConfig.rc_smoothing_mode = 1;
	rxConfig.rc_smoothing_setpoint_cutoff = 0;
	rxConfig.rc_smoothing_feedforward_cutoff = 0;
	rxConfig.rc_smoothing_throttle_cutoff = 0;
	rxConfig.rc_smoothing_debug_axis = ROLL;
	rxConfig.rc_smoothing_auto_factor_rpy = 30;
	rxConfig.rc_smoothing_auto_factor_throttle = 30;
	rxConfig.srxl2_unit_id = 1;
	rxConfig.srxl2_baud_fast = true;
	rxConfig.sbus_baud_fast = false;
	rxConfig.crsf_use_rx_snr = false;
	rxConfig.msp_override_channels_mask = 0;
	rxConfig.crsf_use_negotiated_baud = false;

#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234", &rxConfig);
#else
    parseRcChannels("AETR1234", &rxConfig);
#endif
}


rxChannelRangeConfig_t rxChannelRangeConfigs[NON_AUX_CHANNEL_COUNT];

void rxChannelRangeConfigs_Init(void)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = 989;
        rxChannelRangeConfigs[i].max = 2012;
    }
}

rxFailsafeChannelConfig_t rxFailsafeChannelConfigs[MAX_SUPPORTED_RC_CHANNEL_COUNT];

void rxFailsafeChannelConfigs_Init(void)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig) {
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfig->min = PWM_RANGE_MIN;
        rxChannelRangeConfig->max = PWM_RANGE_MAX;
        rxChannelRangeConfig++;
    }
}

static float nullReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    UNUSED(channel);

    return 0; //PPM_RCVR_TIMEOUT
}

static uint8_t nullFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return RX_FRAME_PENDING;
}

static bool nullProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return true;
}

bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig.rx_min_usec &&
            pulseDuration <= rxConfig.rx_max_usec;
}

#define USE_SERIAL_RX
//#define USE_SERIALRX_SBUS
#define USE_SERIALRX_CRSF

static bool serialRxInit(rxRuntimeState_t *rxRuntimeState)
{
    bool enabled = false;
	enabled = crsfRxInit(rxRuntimeState);
    return enabled;
}

#ifdef _USE_HW_CLI
static void cliRx(cli_args_t *args);
#endif

void rxInit(void)
{
    rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;

    rxRuntimeState.serialrxProvider = rxConfig.serialrx_provider;
    rxRuntimeState.rcReadRawFn = nullReadRawRC;
    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeState.rcProcessFrameFn = nullProcessFrame;
    rxRuntimeState.lastRcFrameTimeUs = 0;
    rcSampleIndex = 0;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig.midrc;
        validRxSignalTimeout[i] = millis() + MAX_INVALID_PULSE_TIME_MS;
    }

    rcData[THROTTLE] = rxConfig.rx_min_usec;

	const bool enabled = serialRxInit(&rxRuntimeState);
	if (!enabled) {
		rxRuntimeState.rcReadRawFn = nullReadRawRC;
		rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
	}

    // Setup source frame RSSI filtering to take averaged values every FRAME_ERR_RESAMPLE_US
    pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig.rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US/1000000.0));

    rxChannelCount = MIN(14 + NON_AUX_CHANNEL_COUNT, rxRuntimeState.channelCount);
    
    #ifdef _USE_HW_CLI
        cliAdd("rx", cliRx);
    #endif
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

//bool rxUpdateCheck()
//{
//    return taskUpdateRxMainInProgress() || rxDataProcessingRequired || auxiliaryProcessingRequired;
//}

void rxFrameCheck(uint32_t currentTimeUs)
{
	bool signalReceived = false;
	bool useDataDrivenProcessing = true;
	int32_t needRxSignalMaxDelayUs = DELAY_100_MS;

	const uint8_t frameStatus = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
	signalReceived = (frameStatus & RX_FRAME_COMPLETE);

    if (signalReceived) {
        //  true only when a new packet arrives
        needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        rxSignalReceived = true; // immediately process packet data
        if (useDataDrivenProcessing) {
            rxDataProcessingRequired = true;
            //  process the new Rx packet when it arrives
        }
    } else {
        //  watch for next packet
        if (cmpTimeUs(currentTimeUs, needRxSignalBefore) > 0) {
            //  initial time to signalReceived failure is 100ms, then we check every 100ms
            rxSignalReceived = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            //  review and process rcData values every 100ms in case failsafe changed them
            rxDataProcessingRequired = true;
        }
    }
}

typedef enum {
    RX_STATE_CHECK,
    RX_STATE_MODES,
    RX_STATE_UPDATE,
    RX_STATE_COUNT
} rxState_e;

static rxState_e rxState = RX_STATE_CHECK;

void taskUpdateRxMain(uint32_t currentTimeUs)
{
    // Where we are using a state machine call schedulerIgnoreTaskExecRate() for all states bar one
    if (rxState != RX_STATE_UPDATE) {
        //schedulerIgnoreTaskExecRate();
    }

    switch (rxState) {
    default:
    case RX_STATE_CHECK:
        if (!processRx(currentTimeUs)) {
            rxState = RX_STATE_CHECK;
            break;
        }
        rxState = RX_STATE_MODES;
        break;

    case RX_STATE_MODES:
        processRxModes(currentTimeUs);
        rxState = RX_STATE_UPDATE;
        break;

    case RX_STATE_UPDATE:
        // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
        updateRcCommands();
        //updateArmingStatus();

        rxState = RX_STATE_CHECK;
        break;
    }
}

#define PPM_RCVR_TIMEOUT            0

static float applyRxChannelRangeConfiguraton(float sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRangef(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    // out of range channel values are now constrained after the validity check in detectAndApplySignalLossBehaviour()
    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxChannelCount; channel++) {

        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig.rcmap[channel] : channel;

        // sample the channel
        float sample;
        sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, &rxChannelRangeConfigs[channel]);
        }

        rcRaw[channel] = sample;
    }
}

void detectAndApplySignalLossBehaviour(void)
{
    const uint32_t currentTimeMs = millis();
    //const bool failsafeAuxSwitch = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    rxFlightChannelsValid = rxSignalReceived;// && !failsafeAuxSwitch;
    //  set rxFlightChannelsValid false when a packet is bad or we use a failsafe switch

    for (int channel = 0; channel < rxChannelCount; channel++) {
        float sample = rcRaw[channel]; // sample has latest RC value, rcData has last 'accepted valid' value
        const bool thisChannelValid = rxFlightChannelsValid && isPulseValid(sample);
        // if the whole packet is bad, consider all channels bad

        if (thisChannelValid) {
            //  reset the invalid pulse period timer for every good channel
            validRxSignalTimeout[channel] = currentTimeMs + MAX_INVALID_PULSE_TIME_MS;
        }

		if (!thisChannelValid) {
			if (cmp32(currentTimeMs, validRxSignalTimeout[channel]) < 0) {
			// first 300ms of Stage 1 failsafe
			sample = rcData[channel];
			//  HOLD last valid value on bad channel/s for MAX_INVALID_PULSE_TIME_MS (300ms)
			} else {
		// remaining Stage 1 failsafe period after 300ms
				if (channel < NON_AUX_CHANNEL_COUNT) {
					rxFlightChannelsValid = false;
					//  declare signal lost after 300ms of any one bad flight channel
				}
				sample = rcData[channel];
					// set channels that are invalid for more than 300ms to Stage 1 values
			}
		}

        sample = constrainf(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

		//  set rcData to either validated incoming values, or failsafe-modified values
		rcData[channel] = sample;
    }
}

bool calculateRxChannelsAndUpdateFailsafe(uint32_t currentTimeUs)
{
    readRxChannelsApplyRanges();            // returns rcRaw
    detectAndApplySignalLossBehaviour();    // returns rcData

    rcSampleIndex++;

    return true;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

void setRssiDirect(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssi = newRssi;
}

#define RSSI_SAMPLE_COUNT 16

static uint16_t updateRssiSamples(uint16_t value)
{
    static uint16_t samples[RSSI_SAMPLE_COUNT];
    static uint8_t sampleIndex = 0;
    static unsigned sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % RSSI_SAMPLE_COUNT;
    return sum / RSSI_SAMPLE_COUNT;
}

void setRssi(uint16_t rssiValue, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    // Filter RSSI value
    if (source == RSSI_SOURCE_FRAME_ERRORS) {
        rssi = pt1FilterApply(&frameErrFilter, rssiValue);
    } else {
        // calculate new sample mean
        rssi = updateRssiSamples(rssiValue);
    }
}

void setRssiMsp(uint8_t newMspRssi)
{
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_MSP;
    }

    if (rssiSource == RSSI_SOURCE_MSP) {
        rssi = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

static void updateRSSIPWM(void)
{
    // Read value of AUX channel as rssi
    int16_t pwmRssi = rcData[rxConfig.rssi_channel - 1]; //.rssi_channel = 0

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}

void updateRSSI(uint32_t currentTimeUs)
{
    switch (rssiSource) {
    case RSSI_SOURCE_RX_CHANNEL:
        updateRSSIPWM();
        break;
    case RSSI_SOURCE_ADC:
        //updateRSSIADC(currentTimeUs);
        break;
    case RSSI_SOURCE_MSP:
        if (cmpTimeUs(micros(), lastMspRssiUpdateUs) > DELAY_1500_MS) {  // 1.5s
            rssi = 0;
        }
        break;
    default:
        break;
    }
}

uint16_t getRssi(void)
{
    uint16_t rssiValue = rssi;

    // RSSI_Invert option
    if (0) { //.rssi_invert = 0
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

    return 100 / 100.0f * rssiValue + 0 * RSSI_OFFSET_SCALING; //.rssi_scale = 100, .rssi_offset = 0
}

uint8_t getRssiPercent(void)
{
    return scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 100);
}

int16_t getRssiDbm(void)
{
    return rssiDbm;
}

#define RSSI_SAMPLE_COUNT_DBM 16

static int16_t updateRssiDbmSamples(int16_t value)
{
    static int16_t samplesdbm[RSSI_SAMPLE_COUNT_DBM];
    static uint8_t sampledbmIndex = 0;
    static int sumdbm = 0;

    sumdbm += value - samplesdbm[sampledbmIndex];
    samplesdbm[sampledbmIndex] = value;
    sampledbmIndex = (sampledbmIndex + 1) % RSSI_SAMPLE_COUNT_DBM;
    return sumdbm / RSSI_SAMPLE_COUNT_DBM;
}

void setRssiDbm(int16_t rssiDbmValue, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssiDbm = updateRssiDbmSamples(rssiDbmValue);
}

void setRssiDbmDirect(int16_t newRssiDbm, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssiDbm = newRssiDbm;
}

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeState.rxRefreshRate;
}

bool isRssiConfigured(void)
{
    return rssiSource != RSSI_SOURCE_NONE;
}

int32_t rxGetFrameDelta(int32_t *frameAgeUs)
{
    static uint32_t previousFrameTimeUs = 0;
    static int32_t frameTimeDeltaUs = 0;

    if (rxRuntimeState.rcFrameTimeUsFn) {
        const uint32_t frameTimeUs = rxRuntimeState.rcFrameTimeUsFn();

        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);

        const int32_t deltaUs = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if (deltaUs) {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }

    return frameTimeDeltaUs;
}

uint32_t rxFrameTimeUs(void)
{
    return rxRuntimeState.lastRcFrameTimeUs;
}

#ifdef _USE_HW_CLI
void cliRx(cli_args_t *args)
{
  bool ret = false;

if (args->argc == 1 && args->isStr(0, "show") == true)
  {
    uint32_t pre_time;
 	pre_time = millis();
    while(cliKeepLoop())
    {
        //scheduler();
        if (millis()-pre_time >= 1000)
    	{
     		pre_time = millis();
            cliPrintf("rx: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n\r", rcRaw[0], rcRaw[1], rcRaw[2], rcRaw[3], rcRaw[4],
            rcRaw[5], rcRaw[6], rcRaw[7], rcRaw[8], rcRaw[9]);
    	}
    }
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("Rx show \n\r");
  }
}
#endif
