/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "accgyro/bmi270.h"
#include "barometer/barometer_dps310.h"
#include "compass/compass_qmc5883l.h"
#include "adc/adcinternal.h"
#include "adc/battery.h"
#include "adc/current.h"
#include "adc/voltage.h"
#include "hw/tim.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/stats.h"
#include "flight/pid.h"
#include "flight/core.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "pg/parameter.h"
#include "msp/msp_box.h"
#include "rx/rc.h"
#include "rx/rc_modes.h"
#include "rx/rc_controls.h"

#include "rx/rx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t time_temp = 0;
uint32_t time_max = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void hwInit(void);
void init(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t telemetry_tx_buf[40];
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  hwInit();
  init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  uint32_t schedule_mpu_gyro = 312;//312us/3.2KHz
  uint32_t schedule_mpu_gyro_temp = micros();

  uint32_t schedule_pid = 312;//312us/3.2KHz
  uint32_t schedule_pid_temp = micros();

  uint32_t schedule_mpu_acc = 1250;//1250us/800Hz
  uint32_t schedule_mpu_acc_temp = micros();

  uint32_t schedule_imu = TASK_PERIOD_HZ(500);
  uint32_t schedule_imu_temp = micros();

  uint32_t schedule_adc = TASK_PERIOD_HZ(1);
  uint32_t schedule_adc_temp = micros();

  uint32_t schedule_baro = TASK_PERIOD_HZ(20);
  uint32_t schedule_baro_temp = micros();

  uint32_t schedule_altitude = TASK_PERIOD_HZ(40);
  uint32_t schedule_altitude_temp = micros();

  uint32_t schedule_rx = TASK_PERIOD_HZ(33);
  uint32_t schedule_rx_temp = micros();

#ifdef USE_MAG_QMC5883
  uint32_t schedule_mag = TASK_PERIOD_HZ(20);
  uint32_t schedule_mag_temp = micros();
#endif

  uint32_t schedule_bat_alerts = TASK_PERIOD_HZ(5);
  uint32_t schedule_bat_alerts_temp = micros();

  uint32_t schedule_bat_volt = TASK_PERIOD_HZ(50);
  uint32_t schedule_bat_volt_temp = micros();

  uint32_t schedule_bat_current = TASK_PERIOD_HZ(50);
  uint32_t schedule_bat_current_temp = micros();

  uint32_t schedule_led = TASK_PERIOD_HZ(100);
  uint32_t schedule_led_temp = micros();

  uint32_t schedule_debug = TASK_PERIOD_HZ(50);
  uint32_t schedule_debug_temp = micros();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(micros()-schedule_mpu_gyro_temp >= schedule_mpu_gyro)
	  {
		  schedule_mpu_gyro_temp = micros();
		  gyroUpdate();
		  time_temp = micros()-schedule_mpu_gyro_temp;
		  if(time_temp >= time_max){
			  time_max = time_temp;
		  }
	  }

	  if(micros()-schedule_pid_temp >= schedule_pid)
	  {
		  schedule_pid_temp = micros();
		  MainPidLoop(schedule_pid_temp);
	  }

	  if(micros()-schedule_mpu_acc_temp >= schedule_mpu_acc)
	  {
		  schedule_mpu_acc_temp = micros();
		  accUpdate();
	  }

	  if(micros()-schedule_imu_temp >= schedule_imu)
	  {
		  schedule_imu_temp = micros();
		  imuUpdateAttitude(schedule_imu_temp);
	  }

	  if(micros()-schedule_adc_temp >= schedule_adc)
	  {
		  schedule_adc_temp = micros();
		  adcInternalProcess();
	  }

	  if(micros()-schedule_baro_temp >= schedule_baro)
	  {
		  schedule_baro_temp = micros();
	      const uint32_t newDeadline = baroUpdate(schedule_baro_temp);
	      if (newDeadline != 0) {
	    	  schedule_baro_temp += newDeadline;
	      }
	  }

	  if(micros()-schedule_altitude_temp >= schedule_altitude)
	  {
		  schedule_altitude_temp = micros();
		  calculateEstimatedAltitude(schedule_altitude_temp);
	  }

	  if(micros()-schedule_rx_temp >= schedule_rx)
	  {
		  schedule_rx_temp = micros();
		  taskUpdateRxMain(schedule_rx_temp);
		  //rxUpdateCheck();
	  }
#ifdef USE_MAG_QMC5883
	  if(micros()-schedule_mag_temp >= schedule_mag)
	  {
		  schedule_mag_temp = micros();
	      const uint32_t newDeadline = compassUpdate(schedule_mag_temp);
	      if (newDeadline != 0) {
	    	  schedule_mag_temp += newDeadline;
	      }
	  }
#endif

	  if(micros()-schedule_bat_alerts_temp >= schedule_bat_alerts)
	  {
		  schedule_bat_alerts_temp = micros();
		  taskBatteryAlerts(schedule_bat_alerts_temp);
	  }

	  if(micros()-schedule_bat_volt_temp >= schedule_bat_volt)
	  {
		  schedule_bat_volt_temp = micros();
		  batteryUpdateVoltage(schedule_bat_volt_temp);
	  }

	  if(micros()-schedule_bat_current_temp >= schedule_bat_current)
	  {
		  schedule_bat_current_temp = micros();
		  batteryUpdateCurrentMeter(schedule_bat_current_temp);

	  }

	  if(micros()-schedule_led_temp >= schedule_led)
	  {
		  schedule_led_temp = micros();
		  static uint32_t pre_time = 0;
		  if(schedule_led_temp - pre_time >= 1000000)
		  {
			pre_time = schedule_led_temp;
			  LED0_TOGGLE;
		  }
	  }

	  if(micros()-schedule_debug_temp >= schedule_debug)
	  {
		  schedule_debug_temp = micros();
//		  cliPrintf("ACC R: %4.d, P: %4.d, Y: %4.d\n\r", mpu.acc.ADCRaw[X], mpu.acc.ADCRaw[Y], mpu.acc.ADCRaw[Z]);
//		  cliPrintf("ACC R: %4.f, P: %4.f, Y: %4.f\n\r", mpu.acc.accADCf[X], mpu.acc.accADCf[Y], mpu.acc.accADCf[Z]);
//		  cliPrintf("GYRO R: %d, P: %d, Y: %d\n\r", mpu.gyro.ADCRaw[X], mpu.gyro.ADCRaw[Y], mpu.gyro.ADCRaw[Z]);
//		  cliPrintf("MAG R: %4.f, P: %4.f, Y: %4.f\n\r", mag.magADC[X], mag.magADC[Y], mag.magADC[Z]);
		  cliPrintf("IMU R: %4.d, P: %4.d, Y: %4.d\n\r",    attitude.values.roll,
													        attitude.values.pitch,
													        attitude.values.yaw);
//		  cliPrintf("BARO : %d cm \n\r", baro.BaroAlt);
//		  cliPrintf("rx 1: %.1f, 2: %.1f, 3: %.1f, 4: %.1f, 5: %.1f\n\r", rcRaw[0],rcRaw[1],rcRaw[2],rcRaw[3],rcRaw[4]);
//	  	  telemetry_tx_buf[0] = 0x46;
//	  	  telemetry_tx_buf[1] = 0x43;
//
//	  	  telemetry_tx_buf[2] = 0x10;
//
//	  	  telemetry_tx_buf[3] = (short)(mpu.gyro.gyroADC[X]*10);
//	  	  telemetry_tx_buf[4] = ((short)(mpu.gyro.gyroADC[X]*10))>>8;
//
//	  	  telemetry_tx_buf[5] = (short)(mpu.gyro.gyroADC[Y]*10);
//	  	  telemetry_tx_buf[6] = ((short)(mpu.gyro.gyroADC[Y]*10))>>8;
//
//	  	  telemetry_tx_buf[7] = (short)(mpu.gyro.gyroADC[Z]*10);
//	  	  telemetry_tx_buf[8] = ((short)(mpu.gyro.gyroADC[Z]*10))>>8;
//
//	  	  telemetry_tx_buf[9] = (short)(baro.BaroAlt*10);
//	  	  telemetry_tx_buf[10] = ((short)(baro.BaroAlt*10))>>8;
//
//		  telemetry_tx_buf[11] = (short)(mpu.gyro.gyroADCf[X]*10);
//		  telemetry_tx_buf[12] = ((short)(mpu.gyro.gyroADCf[X]*10))>>8;
//
//		  telemetry_tx_buf[13] = (short)(mpu.gyro.gyroADCf[Y]*10);
//		  telemetry_tx_buf[14] = ((short)(mpu.gyro.gyroADCf[Y]*10))>>8;
//
//		  telemetry_tx_buf[15] = (short)(mpu.gyro.gyroADCf[Z]*10);
//		  telemetry_tx_buf[16] = ((short)(mpu.gyro.gyroADCf[Z]*10))>>8;
//
//	  	  telemetry_tx_buf[17] = 0x00;
//	  	  telemetry_tx_buf[18] = 0x00;
//
//	  	  telemetry_tx_buf[19] = 0xff;
//
//	  	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
//
//	  	  uartWrite(0, &telemetry_tx_buf[0], 20);
	  }
	  rxFrameCheck(micros());
	  cliMain();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void hwInit(void)
{
  #ifdef _USE_HW_RTC
    rtcInit();
  #endif
  gpioInit();
  flashInit();
  ledInit();
  MX_DMA_Init();
  usbInit();
  uartInit();
  cliInit();
  i2cInit();
  spiInit();
  adcInit();
  TIM_Init();

  if (sdInit() == true)
  {
    fatfsInit();
  }
}

void init(void)
{

    imuConfig_Init();
	rxConfig_Init();
	rxChannelRangeConfigs_Init();
	rxFailsafeChannelConfigs_Init();
	statsConfig_Init();
	armingConfig_Init();
	pidProfile_Init();
	mixerConfig_Init();
	motorConfig_Init();
#ifdef USE_MAG_QMC5883
	compassConfig_Init();
#endif
	flight3DConfig_Init();
	positionConfig_Init();
	adcConfig_Init();
	voltageSensorADCConfig_Init();
	currentSensorADCConfig_Init();
	batteryConfig_Init();
	pidConfig_Init();

	initRcProcessing();
	rcControlsConfig_Init();
	rcControlsInit();
	initActiveBoxIds();

	cliOpen(_DEF_USB, 57600);

	bmi270_Init();
	dps310_Init();
#ifdef USE_MAG_QMC5883
	compassInit();
#endif
	adcInternalInit();
	LED1_ON;
	for (int i = 0; i < 10; i++)
	{
		LED0_TOGGLE;
		#if defined(USE_BEEPER)
		delay(25);
		if (!(beeperConfig.beeper_off_flags & BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT))) {
			BEEP_ON;
		}
			delay(25);
			BEEP_OFF;
		#else
			delay(50);
		#endif
	}
	LED0_OFF;
	////////////////////////////////////////

	imuInit();
	mixerInit(mixerConfig.mixerMode);

	uint16_t idlePulse = motorConfig.mincommand;

	/* Motors needs to be initialized soon as posible because hardware initialization
	* may send spurious pulses to esc's causing their early initialization. Also ppm
	* receiver may share timer with motors so motors MUST be initialized here. */
	motorDevInit(&motorConfig.dev, idlePulse, getMotorCount());

	rxInit();

    gyroStartCalibration(false);
    baroStartCalibration();

	batteryInit(); // always needs doing, regardless of features.

	MSP_SET_MODE_RANGE(0,  0, 0, 1700, 2100);
	MSP_SET_MODE_RANGE(1,  1, 1,  900, 2100);
	MSP_SET_MODE_RANGE(2,  6, 2, 1300, 2100);
	MSP_SET_MODE_RANGE(3, 27, 4, 1700, 2100);
	MSP_SET_MODE_RANGE(4,  7, 5, 1700, 2100);
	MSP_SET_MODE_RANGE(5, 13, 5, 1700, 2100);
	MSP_SET_MODE_RANGE(6, 39, 4, 1700, 2100);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
