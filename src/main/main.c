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
#include "accgyro/bmi270.h"
#include "barometer/barometer_dps310.h"
#include "compass/compass_qmc5883l.h"
#include "adc/adcinternal.h"
#include "hw/tim.h"
#include "flight/imu.h"
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
flags_t f;
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

  uint32_t schedule_mpu_acc = 1250;//1250us/800Hz
  uint32_t schedule_mpu_acc_temp = micros();

  uint32_t schedule_imu = TASK_PERIOD_HZ(100);//1250us/800Hz
  uint32_t schedule_imu_temp = micros();

  uint32_t schedule_adc = TASK_PERIOD_HZ(1);//1250us/800Hz
  uint32_t schedule_adc_temp = micros();

  uint32_t schedule_baro = TASK_PERIOD_HZ(20);//1250us/800Hz
  uint32_t schedule_baro_temp = micros();

  uint32_t schedule_mag = TASK_PERIOD_HZ(20);//1250us/800Hz
  uint32_t schedule_mag_temp = micros();

  uint32_t schedule_led = TASK_PERIOD_HZ(100);//1250us/800Hz
  uint32_t schedule_led_temp = micros();

  uint32_t schedule_debug = TASK_PERIOD_HZ(50);//1250us/800Hz
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

	  if(micros()-schedule_mag_temp >= schedule_mag)
	  {
		  schedule_mag_temp = micros();
//	      const uint32_t newDeadline = compassUpdate(schedule_mag_temp);
//	      if (newDeadline != 0) {
//	    	  schedule_mag_temp += newDeadline;
//	      }
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
		  //cliPrintf("ACC R: %d, P: %d, Y: %d\n\r", mpu.acc.ADCRaw[X], mpu.acc.ADCRaw[Y], mpu.acc.ADCRaw[Z]);
		  //cliPrintf("GYRO R: %d, P: %d, Y: %d\n\r", mpu.gyro.ADCRaw[X], mpu.gyro.ADCRaw[Y], mpu.gyro.ADCRaw[Z]);
//		  cliPrintf("IMU R: %d, P: %d, Y: %d\n\r",    attitude.values.roll,
//													  attitude.values.pitch,
//													  attitude.values.yaw);
		  cliPrintf("BARO : %d cm \n\r", baro.BaroAlt);
	  }

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
