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
typedef e_sm_State (*f_sm_Handler)(void);

typedef struct
{
	e_sm_State srcState;
	e_sm_Event event;
	e_sm_State dstState;
}t_sm_Transition;


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
bool eventFlag;

t_pid_Control PidCtrl;
t_pid_Parameter PidParam;

uint32_t dwtCycles;

e_sm_State f_sm_Error();
e_sm_State f_sm_Init();
e_sm_State f_sm_Idle();
e_sm_State f_sm_Work();
e_sm_State f_sm_Exit();

static const t_sm_Transition SM_Transition[] =
{
		{ST_ERROR, EV_ERROR, ST_ERROR},
		{ST_ERROR, EV_NO_EVENT, ST_ERROR},
		{ST_ERROR, EV_BUTTON_A, ST_ERROR},
		{ST_ERROR, EV_BUTTON_B, ST_ERROR},
		{ST_INIT, EV_ERROR, ST_ERROR},
		{ST_INIT, EV_NO_EVENT, ST_IDLE}
};

t_sm_Transition SM;

static const f_sm_Handler StateHandler[] = {f_sm_Error, f_sm_Init, f_sm_Idle, f_sm_Work, f_sm_Exit};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t f_dwt_counterEnable();
static inline void f_dwt_addSample();



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  SM.dstState = ST_INIT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  SM.srcState = SM.dstState;
	  if(StateHandler[SM.srcState] != NULL)
	  {
		  SM.dstState = (StateHandler[SM.srcState])();
	  }
	  else
	  {
		  // Invalid code
		  break;
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

e_sm_State f_sm_Error()
{
	__disable_irq();
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

	while(1)
		;

	return ST_ERROR;
}

e_sm_State f_sm_Init()
{
	static const char *infoTxt[] =
	{
		"Init test",
		"Test ok",
		"Test failure"
	};

	uint8_t isOkCounter = 0;

	f_lcd_Init();
	f_work_MotorInitTimer();
	f_work_SensorInitTimer();
	f_ina219_Init();

	f_lcd_ClearAll();
	f_lcd_WriteTxt(0, 16, infoTxt[0], &font_msSansSerif_14);
	HAL_Delay(500);

	//check motor on idle
	f_work_MotorSet(0);
	isOkCounter += f_work_MotorTest(0);

	//check motor on 50%
	f_work_MotorSetVelocity(MAX_MOTOR_PWM/2);
	f_work_MotorSet(1);
	HAL_Delay(2000);
	isOkCounter += f_work_MotorTest(1);
	f_work_MotorSet(0);

	//check is distance is not 0
	f_work_sensorTriggerMeasure();
	while(!sensorMeasureDone)
		;
	uint16_t distance = f_work_sensorGetLastMeasure();
	if(distance > 0) isOkCounter++;

	f_lcd_ClearAll();
	if(isOkCounter == 3)
	{
		f_lcd_WriteTxt(0, 16, infoTxt[1], &font_msSansSerif_14);
		return ST_IDLE;
	}
	else
		{
			f_lcd_WriteTxt(0, 16, infoTxt[2], &font_msSansSerif_14);
			return ST_ERROR;
		}
}

e_sm_State f_sm_Idle()
{
	static enum {PREPARE, SET_P, SET_I, SET_D, TEST} Substate;
	static const char *infoTxt[] =
	{
		"Set PID values:",
		"Set P value:"
		"Set I value:",
		"Set D value:",
	};
	char txt[20];

	switch (Substate)
	{
		case PREPARE:
			f_lcd_ClearAll();
			f_lcd_WriteTxt(0, 16, infoTxt[0], &font_msSansSerif_14);

			break;

		case SET_P:
			float lastVal;
			f_lcd_WriteTxt(0, 32, infoTxt[1], &font_msSansSerif_14);

			while(!eventFlag)
			{
				if(lastVal != PidParam.Kp)
				{
					sprintf(txt, "%.2f", PidParam.Kp);
					f_lcd_Clear(0, 127, 4);
					f_lcd_Clear(0, 127, 5);
					f_lcd_WriteTxt(0, 32, txt, &font_msSansSerif_14);
				}
			}

		default:
			break;
	}
}

e_sm_State f_sm_Work()
{

}

e_sm_State f_sm_Exit()
{

}

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
