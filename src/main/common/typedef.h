/*
 * Typedef.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_TYPEDEF_H_
#define SRC_COMMON_CORE_TYPEDEF_H_


#include "hw.h"

//This is the mode what is selected via the remote (NONE, HOLD, RTH and NAV (NAV-> exectute mission)
enum gpsmode {
  GPS_MODE_NONE = 0,
  GPS_MODE_HOLD,
  GPS_MODE_RTH,
  GPS_MODE_NAV
};

enum navstate {
  NAV_STATE_NONE = 0,
  NAV_STATE_RTH_START,
  NAV_STATE_RTH_ENROUTE,
  NAV_STATE_HOLD_INFINIT,
  NAV_STATE_HOLD_TIMED,
  NAV_STATE_WP_ENROUTE,
  NAV_STATE_PROCESS_NEXT,
  NAV_STATE_DO_JUMP,
  NAV_STATE_LAND_START,
  NAV_STATE_LAND_IN_PROGRESS,
  NAV_STATE_LANDED,
  NAV_STATE_LAND_SETTLE,
  NAV_STATE_LAND_START_DESCENT
};

enum naverror {
  NAV_ERROR_NONE = 0,            //All systems clear
  NAV_ERROR_TOOFAR,              //Next waypoint distance is more than safety distance
  NAV_ERROR_SPOILED_GPS,         //GPS reception is compromised - Nav paused - copter is adrift !
  NAV_ERROR_WP_CRC,              //CRC error reading WP data from EEPROM - Nav stopped
  NAV_ERROR_FINISH,              //End flag detected, navigation finished
  NAV_ERROR_TIMEWAIT,            //Waiting for poshold timer
  NAV_ERROR_INVALID_JUMP,        //Invalid jump target detected, aborting
  NAV_ERROR_INVALID_DATA,        //Invalid mission step action code, aborting, copter is adrift
  NAV_ERROR_WAIT_FOR_RTH_ALT,    //Waiting to reach RTH Altitude
  NAV_ERROR_GPS_FIX_LOST,        //Gps fix lost, aborting mission
  NAV_ERROR_DISARMED,            //NAV engine disabled due disarm
  NAV_ERROR_LANDING              //Landing
};

typedef struct flags_t {
    uint8_t OK_TO_ARM;
   	uint8_t ARMED;
	//////////////////////
    uint8_t Tuning_MODE;
    uint8_t Write_MODE;
    ///////////////////////
    uint8_t CALIBRATE_ACC;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t ACRO_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
	  uint8_t User_MODE;
	  uint8_t GPS_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLE;
    uint8_t CALIBRATE_MAG;
    uint8_t VARIO_MODE;
    uint8_t FIXED_WING;                     // set when in flying_wing or airplane mode. currently used by althold selection code
    uint8_t MOTORS_STOPPED;
    uint8_t FW_FAILSAFE_RTH_ENABLE;
    uint8_t CLIMBOUT_FW;
    uint8_t CRUISE_MODE;
    uint8_t mag_reset;
} flags_t;

enum box {
  BOXARM,
  BOXHEADFREE,
  BOXACRO_MODE,
  BOXANGLE_MODE,
  BOXCALIBRATE_ACC,
  BOXCALIBRATE_MAG,
  BOXGPS_MODE,
  BOXGPSHOME,
  BOXGPSHOLD,
  BOXGPSNAV,
  BOXLAND,
  CHECKBOXITEMS
};

typedef struct eeror_t {
  uint8_t error;
  uint8_t error_counter;
  uint8_t error_led;
  uint32_t error_timer;
} eeror_t;



#endif /* SRC_COMMON_CORE_TYPEDEF_H_ */
