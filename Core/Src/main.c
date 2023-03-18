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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "state_machine.h"
#include "ina219.h"
#include "lcd_service.h"

#include "sh1106_hw.h"
#include "work.h"
#include "pid.h"
#include "gui.h"
#include "circ_buff.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define f_dwt_startMeasure()		DWT->CYCCNT = 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const t_smStateMachine ProgramStates[] = {};
int16_t current;
int16_t shunt;
uint16_t bus;
uint16_t power;
uint16_t distance;
uint32_t dwtCycles;

t_pid_Control PidCtrl;
t_pid_Parameter Param = {
		10, 1, 2, -900, 900, -4000,4000
};

int32_t throttle;
int16_t pwm;

uint8_t chartData[120];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t f_dwt_counterEnable()
{
	uint32_t cycle;

	CoreDebug->DEMCR &= ~(1<<24);
	CoreDebug->DEMCR |= (1<<24);

	DWT->CTRL &= ~(1<<0);
	DWT->CTRL |= (1<<0);

	DWT->CYCCNT = 0;

	cycle = DWT->CYCCNT;

	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	if((DWT->CYCCNT - cycle) == 0) return 0;

	return (DWT->CYCCNT - cycle);
}

static inline void f_dwt_addSample()
{
	static uint32_t cycleAccum;
	static uint8_t dwtSamples;

	cycleAccum += DWT->CYCCNT;
	dwtSamples++;

	if(dwtSamples == 32)
	{
		dwtCycles = cycleAccum / 32;
		cycleAccum = 0;
		dwtSamples = 0;
	}
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

   f_work_motorInitTimer();
   f_work_sensorInitTimer();
   f_ina219_Init();
   f_lcd_Init();

   char txt[20];

   uint32_t timerLCD;
   uint32_t timerCTRL;
   uint32_t timerPID;

   uint8_t iterator = 0;
   uint8_t totalLength = 0;


   f_gui_drawChart(chartData, 0, 0);
   f_work_motorSet(1);
   f_dwt_counterEnable();
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//TODO:: first sample of lcd is the current
	  if((HAL_GetTick() - timerCTRL) >= 100)
	  {
		  distance = f_work_sensorGetLastMeasure();
		  f_work_sensorTriggerMeasure();
		  power = f_ina219_GetPowerInMilis()/100;


		  f_pid_calculateThrottle(150, power, &PidCtrl, &Param);

		  pwm += PidCtrl.output/200;
		  if(pwm > 100) pwm = 100;
		  else if(pwm < 0) pwm = 0;

		  chartData[++iterator%120] = pwm / 3;

		  if(totalLength < 120)
		  {
			  f_gui_drawChart(chartData, totalLength, 0);
			  totalLength++;
		  }
		  else
		  {
			  f_dwt_startMeasure();
			  f_gui_drawChart(chartData, 120, iterator);
			  f_dwt_addSample();
		  }
		  //if(distance) f_work_motorSetVelocity(distance/10);

		  f_work_motorSetVelocity(pwm);

		  sprintf(txt, "Pow: %2d.%2d", power/10, power%10);
		  f_lcd_WriteTxt(0, 0, txt, &test2);

		  timerCTRL = HAL_GetTick();
	  }



  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
