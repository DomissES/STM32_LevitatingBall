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
#include <math.h>
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
typedef e_sm_ReturnCode (*f_sm_Handler)(void);

typedef struct
{
	e_sm_State srcState;
	e_sm_Event event;
	e_sm_State dstState;
}t_sm_Transition;

struct
{
	bool counterEnableA;
	bool counterEnableB;
	uint32_t counterA;
	uint32_t counterB;
}Button;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define f_dwt_startMeasure()			DWT->CYCCNT = 0

#define BUTTON_PRESSED_CHECKOUT_TIME	100 //ms
#define PINGPONG_MIN_DISTANCE			50 //mm
#define PINGPONG_MAX_DISTANCE			450 //mm
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
e_sm_State f_sm_Replay();
e_sm_State f_sm_Exit();

t_sm_Transition SM;

static const f_sm_Handler StateHandler[] = {f_sm_Error, f_sm_Init, f_sm_Idle, f_sm_Work, f_sm_Replay, f_sm_Exit};
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
	f_lcd_ClearAll();
	f_gui_DrawHeading(SM.srcState, LCD_ERROR);
	f_lcd_WriteTxt(0, 32, "ERROR!", &font_msSansSerif_14);

	while(1)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_Delay(500);
	}

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
	e_sm_State nextState;
	uint8_t isOkCounter = 0;
	//TODO:: WATCHDOG ENABLE;
	//enable all basic peripherals
	f_lcd_Init();
	f_work_MotorInitTimer();
	f_work_SensorInitTimer();
	f_ina219_Init();
	//enable encoder timer
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	//HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);


	f_lcd_ClearAll();
	f_gui_DrawHeading(SM.dstState, LCD_NOPAGE);
	f_lcd_WriteTxt(0, 32, infoTxt[0], &font_msSansSerif_14);
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
		f_lcd_WriteTxt(0, 32, infoTxt[1], &font_msSansSerif_14);
		nextState = ST_IDLE;
	}
	else
	{
		f_lcd_WriteTxt(0, 32, infoTxt[2], &font_msSansSerif_14);
		nextState = ST_ERROR;
	}
	HAL_Delay(1500);

	return nextState;
}

static uint16_t f_idle_GetEncoderInput()
{
	char txt[20];
	uint16_t lastVal;
	uint16_t *timerCounter = (uint16_t*)&htim3.Instance->CNT;
	*timerCounter = 0;

	while(!eventFlag)
	{
		if(lastVal != *timerCounter)
		{
			uint16_t tmpVal = *timerCounter/2;
			sprintf(txt, "%2d.%02d", tmpVal/100, tmpVal%100);
			f_lcd_Clear(0, 128, 6);
			f_lcd_Clear(0, 128, 7);
			f_lcd_WriteTxt(0, 48, txt, &font_msSansSerif_14);
			lastVal = *timerCounter;
		}
	}
	return *timerCounter/2;
}

e_sm_State f_sm_Idle()
{
	static enum {PREPARE, SET_P, SET_I, SET_D} Substate;
	e_sm_State nextState;
	static const char *infoTxt[] =
	{
		"Set PID values:\0",
		"Set P value:\0",
		"Set I value:\0",
		"Set D value:\0"
	};

	f_lcd_Clear(0, 128, 4);
	f_lcd_Clear(0, 128, 5);

	switch (Substate)
	{
		case PREPARE:
			f_lcd_ClearAll();
			f_gui_DrawHeading(SM.srcState, LCD_INPUT);
			f_lcd_WriteTxt(0, 16, infoTxt[0], &font_msSansSerif_14);
			Substate = SET_P;
			SM.event = EV_NO_EVENT;
			break;

		case SET_P:
			f_lcd_WriteTxt(0, 32, infoTxt[1], &font_msSansSerif_14);
			PidParam.Kp = (float)f_idle_GetEncoderInput()/100;
			break;

		case SET_I:
			f_lcd_WriteTxt(0, 32, infoTxt[2], &font_msSansSerif_14);
			PidParam.Ki = (float)f_idle_GetEncoderInput()/100;
			break;

		case SET_D:
			f_lcd_WriteTxt(0, 32, infoTxt[3], &font_msSansSerif_14);
			PidParam.Kd = (float)f_idle_GetEncoderInput()/100;

			PidParam.I_maxRange = 1000;
			PidParam.I_minRange = -1000;
			PidParam.maxRange = MAX_MOTOR_PWM/2;
			PidParam.minRange = -(MAX_MOTOR_PWM/2);

			break;

		default:
			nextState = ST_ERROR;
			break;
	}

	nextState = ST_IDLE;
	eventFlag = false;
	if(SM.event == EV_BUTTON_A) Substate++;
	else if(SM.event == EV_BUTTON_B) Substate--;
	else if(SM.event == EV_ERROR) nextState = ST_ERROR;
	SM.event = EV_NO_EVENT;
	if(Substate > SET_D)
		{
			nextState = ST_WORK;
			Substate = PREPARE;
		}

	return nextState;
}

static void f_work_drawPage(e_gui_lcdPage page, t_pid_Parameter *Param, uint16_t pwmOutput, uint16_t distanceSet, uint16_t distanceGet)
{
	static uint8_t chartData[120];
	static uint8_t chartIterator, chartLength;

	switch (page)
	{
		case LCD_PARAM:
			f_gui_DrawParamPage(Param, &PidCtrl);
			break;

		case LCD_CTRL:
			f_gui_DrawCtrlPage((float)distanceSet/10, (float)distanceGet/10, (float)pwmOutput/41);
			break;

		case LCD_CHART:
			chartData[chartIterator] = (uint32_t)(pwmOutput*44)/4096; //max value is 44px
			chartIterator = (chartIterator + 1) % 120;

			if(chartLength < 120)
			{
				f_gui_DrawChartPage(chartData, chartLength, 0);
				chartLength++;
			}
			else
			{
				f_gui_DrawChartPage(chartData, chartLength, chartIterator);
			}

			break;

		default:
			f_lcd_ClearAll();
			break;
	}
}

e_sm_State f_sm_Work()
{
	e_sm_State nextState;
	static enum {PREPARE, WORK, POSTPARE, EXIT} Substate;
	uint32_t timerLcdInput, timerLcdHeading, timerMotor, timerMotorTest;
	uint16_t *distanceSet = (uint16_t*)&htim3.Instance->CNT;
	uint16_t distanceGet, distanceLastSet;
	bool isMotorPowerOk;
	uint16_t motorPwm;
	uint32_t motorPwmDelta;
	static e_gui_lcdPage currentLcdPage = LCD_NOPAGE;
	bool changePage;

	f_work_sensorTriggerMeasure();
	*distanceSet = (PINGPONG_MAX_DISTANCE-PINGPONG_MIN_DISTANCE)/2;


	switch (Substate)
	{
		case PREPARE:
			isMotorPowerOk = f_work_MotorTest(0);
			if(isMotorPowerOk)
			{
				f_lcd_ClearAll();
				f_lcd_WriteTxt(0, 32, "Press OK", &font_msSansSerif_14);

				while(1)
				{
					if(SM.event == EV_BUTTON_A) break;
				}

				currentLcdPage = LCD_PARAM;
			}
			else nextState = ST_ERROR;

			break;

		case WORK:
			f_gui_DrawHeading(ST_WORK, currentLcdPage);
			f_work_MotorSet(1);
			timerLcdInput = timerMotor = timerLcdHeading = timerMotorTest = HAL_GetTick();

			while(1)
			{
				if(((HAL_GetTick() - timerLcdInput) > 200) || (distanceLastSet != *distanceSet/2)) //if timer or input changed
				{
					if(changePage)
					{
						currentLcdPage++;
						if(currentLcdPage == LCD_NOPAGE) currentLcdPage = LCD_PARAM;
						changePage = 0;
					}
					f_work_drawPage(currentLcdPage, &PidParam, motorPwm, *distanceSet, distanceGet);

					if(*distanceSet > PINGPONG_MAX_DISTANCE) *distanceSet = PINGPONG_MAX_DISTANCE;
					else if(*distanceSet < PINGPONG_MIN_DISTANCE) *distanceSet = PINGPONG_MIN_DISTANCE;
					distanceLastSet = *distanceSet/2;

					timerLcdInput = HAL_GetTick();
				}

				if(( HAL_GetTick() - timerLcdHeading) > 500)
				{
					f_gui_DrawHeading(SM.dstState, currentLcdPage);
					timerLcdHeading = HAL_GetTick();
				}

				if((HAL_GetTick() - timerMotor) > 20)
				{
					uint16_t timeout = 20;
					while(!sensorMeasureDone && timeout)
					{
						HAL_Delay(1);
						timeout--;
					}
					distanceGet = f_work_sensorGetLastMeasure(); //in mm

					f_pid_calculateThrottle(*distanceSet, distanceGet, &PidCtrl, &PidParam);
					//motorPwm -= (int16_t)PidCtrl.output; //error is opposite
					motorPwm = MAX_MOTOR_PWM/2 - (int16_t)PidCtrl.output;
					if(motorPwm > MAX_MOTOR_PWM) motorPwm = MAX_MOTOR_PWM;
					else if(motorPwm < 0) motorPwm = 0;

					f_work_MotorSetVelocity(motorPwm);
					motorPwmDelta = (15*motorPwmDelta + motorPwm)/16;

					f_work_sensorTriggerMeasure();
					timerMotor = HAL_GetTick();
				}

				if((HAL_GetTick() - timerMotorTest) > 500)
				{
					if(abs(motorPwmDelta - motorPwm) < (MAX_MOTOR_PWM/100)) // if motorPwm is stabilized
					{
						isMotorPowerOk = f_work_MotorTest(1);
						if((distanceGet == 0) || !isMotorPowerOk)
						{
							f_work_MotorSet(0);
							nextState = ST_ERROR;
							break;
						}
					}
					timerMotorTest = HAL_GetTick();
				}

				if(eventFlag)
				{
					if(SM.event == EV_BUTTON_B)	break;
					else if(SM.event == EV_BUTTON_A) changePage = 1;
					else if(SM.event == EV_ERROR)
					{
						nextState = ST_ERROR;
						break;
					}
					SM.event = EV_NO_EVENT;
					eventFlag = 0;
				}
			}
			break;

		case POSTPARE:
			f_work_MotorSet(0);
			break;

		default:
			break;
	}

	eventFlag = 0;
	Substate++;
	SM.event = EV_NO_EVENT;
	if(Substate == EXIT) nextState = ST_REPLAY;

	return nextState;
}

e_sm_State f_sm_Replay()
{
	e_sm_State nextState;

	f_lcd_ClearAll();
	f_gui_DrawHeading(SM.srcState, LCD_NOPAGE);
	f_lcd_WriteTxt(0, 32, "Replay?", &font_msSansSerif_14);

	while(!eventFlag)
		;

	eventFlag = 0;
	if(SM.event == EV_BUTTON_A)	nextState = ST_IDLE;
	else if(SM.event == EV_BUTTON_B) nextState = ST_EXIT;
	else if(SM.event == EV_ERROR) nextState = ST_ERROR;
	SM.event = EV_NO_EVENT;

	return nextState;
}

e_sm_State f_sm_Exit()
{
	f_lcd_ClearAll();
	f_lcd_WriteTxt(0, 32, "EXIT", &font_msSansSerif_14);

	while(1)
		;

	return ST_ERROR;
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

void f_CheckButtonsCallback()
{
	if(Button.counterEnableA && HAL_GPIO_ReadPin(B_NEXT_GPIO_Port, B_NEXT_Pin))
	{
		if((HAL_GetTick() - Button.counterA) >= BUTTON_PRESSED_CHECKOUT_TIME)
		{
			eventFlag = true;
			SM.event = EV_BUTTON_A;
			Button.counterEnableA = false;
		}
	}
	else Button.counterEnableA = false;

	if(Button.counterEnableB && HAL_GPIO_ReadPin(B_PREV_GPIO_Port, B_PREV_Pin))
	{
		if((HAL_GetTick() - Button.counterB) >= BUTTON_PRESSED_CHECKOUT_TIME)
		{
			eventFlag = true;
			SM.event = EV_BUTTON_B;
			Button.counterEnableB = false;
		}
	}
	else Button.counterEnableB = false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	if((GPIO_Pin == B_NEXT_Pin) && HAL_GPIO_ReadPin(B_NEXT_GPIO_Port, B_NEXT_Pin))
	{
		Button.counterEnableA = true;
		Button.counterA = HAL_GetTick();
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}
	else if((GPIO_Pin == B_PREV_Pin) && HAL_GPIO_ReadPin(B_PREV_GPIO_Port, B_PREV_Pin))
	{
		Button.counterEnableB = true;
		Button.counterB = HAL_GetTick();
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
