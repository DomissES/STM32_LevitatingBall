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
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include "ina219.h"
#include "lcd_service.h"

#include "sh1106_hw.h"
#include "pid.h"
#include "gui.h"
#include "circ_buff.h"
#include "runtime.h"
#include "machine.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef e_sm_State (*f_sm_Handler)(void);

typedef struct
{
	e_sm_State currentState;
	e_sm_Event event;
}t_sm_Transition;

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
bool encoderInputChanged;
uint32_t dwtCycles;

struct
{
	bool counterEnableA;
	bool counterEnableB;
	uint32_t counterA;
	uint32_t counterB;
}Button;

t_pid_Control PidCtrl;
t_pid_Parameter PidParam;

e_sm_State f_sm_Error();
e_sm_State f_sm_Init();
e_sm_State f_sm_Idle();
e_sm_State f_sm_Work();
e_sm_State f_sm_Replay();
e_sm_State f_sm_Exit();

t_sm_Transition SM; //main State Machine Variable

static const f_sm_Handler StateHandler[] = {f_sm_Error, f_sm_Init, f_sm_Idle, f_sm_Work, f_sm_Replay, f_sm_Exit};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t f_dwt_counterEnable();
static inline void f_dwt_addSample();

static inline void f_sm_ClearEventFlag()
{
	eventFlag = false;
	SM.event = EV_NO_EVENT;
}


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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  SM.currentState = ST_INIT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(StateHandler[SM.currentState] != NULL)
	  {
		  SM.currentState = (StateHandler[SM.currentState])();
	  }
	  else// Invalid code
	  {
		  break;
	  }
	  HAL_IWDG_Refresh(&hiwdg);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
	f_gui_DrawHeading(SM.currentState, LCD_ERROR);
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
	static const char *const infoTxt[] =
	{
		"Init test",
		"Test ok",
		"Test failure"
	};
	e_sm_State nextState;
	uint8_t isOkCounter = 0;

	isOkCounter += f_runtime_FirstInit();

	f_lcd_ClearAll();
	f_gui_DrawHeading(SM.currentState, LCD_NOPAGE);
	f_lcd_WriteTxt(0, 32, infoTxt[0], &font_msSansSerif_14);
	HAL_Delay(500);

	isOkCounter += f_runtime_FirstTest();

	f_lcd_ClearAll();
	if(isOkCounter == 2)
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

e_sm_State f_sm_Idle()
{
	static enum {PROLOGUE, SET_P, SET_I, SET_D, EPILOGUE} Substate;
	e_sm_State nextState = ST_IDLE;
	static const char *const infoTxt[] =
	{
		"Set PID values:",
		"Set P value:",
		"Set I value:",
		"Set D value:",
		"Press OK"
	};

	f_lcd_Clear(0, 128, 4);
	f_lcd_Clear(0, 128, 5);

	switch (Substate)
	{
		case PROLOGUE:
			f_gui_DrawHeading(SM.currentState, LCD_INPUT);
			f_lcd_WriteTxt(0, 16, infoTxt[0], &font_msSansSerif_14);
			Substate = SET_P;
			SM.event = EV_NO_EVENT;
			break;

		case SET_P: //could be optimized with SET_I and SET_D, left for better readability what's going on
			f_lcd_WriteTxt(0, 32, infoTxt[1], &font_msSansSerif_14);
			PidParam.Kp = f_runtime_GetParamInput(10);
			break;

		case SET_I:
			f_lcd_WriteTxt(0, 32, infoTxt[2], &font_msSansSerif_14);
			PidParam.Ki = f_runtime_GetParamInput(1);
			break;

		case SET_D:
			f_lcd_WriteTxt(0, 32, infoTxt[3], &font_msSansSerif_14);
			PidParam.Kd = f_runtime_GetParamInput(5);
			break;

		case EPILOGUE:
			f_lcd_ClearAll();
			f_gui_DrawHeading(SM.currentState, LCD_NOPAGE);
			f_lcd_WriteTxt(0, 32, infoTxt[4], &font_msSansSerif_14);

			while(!eventFlag)
				;
			f_sm_ClearEventFlag(); //if not, the EV_BUTTON_A would increase the substate to SET_P
			PidParam.I_maxRange = 1000;
			PidParam.I_minRange = -1000;
			PidParam.maxRange = MAX_MOTOR_PWM/2;
			PidParam.minRange = -(MAX_MOTOR_PWM/2);

			Substate = PROLOGUE;
			nextState = ST_WORK;
			break;

		default:
			break;
	}

	if(SM.event == EV_BUTTON_A) Substate++;
	else if(SM.event == EV_BUTTON_B) Substate--;
	else if(SM.event == EV_ERROR) nextState = ST_ERROR;
	f_sm_ClearEventFlag();

	return nextState;
}

e_sm_State f_sm_Work()
{
	e_sm_State nextState = ST_WORK;
	static enum {PREPARE, WORK, POSTPARE, EXIT} Substate;
	uint32_t timerLcdInput, timerLcdHeading, timerMotor, timerMotorTest;
	//distanceSet is encoder counter address
	uint16_t *distanceSet = (uint16_t*)&htim3.Instance->CNT;
	*distanceSet = (PINGPONG_MAX_DISTANCE-PINGPONG_MIN_DISTANCE)/2;
	uint16_t distanceGet;
	uint16_t motorPwm;
	uint16_t motorPwmMean;//for calculating pwm stability
	static e_gui_lcdPage currentLcdPage = LCD_PARAM;
	bool changePage;

	f_machine_SensorTriggerMeasure();

	switch (Substate)
	{
		case PREPARE:
			if(!f_machine_MotorTestIfOk()) nextState = ST_ERROR;
			break;

		case WORK:
			f_gui_DrawHeading(ST_WORK, currentLcdPage);
			f_machine_MotorSet(1);
			timerLcdInput = timerMotor = timerLcdHeading = timerMotorTest = HAL_GetTick();

			while(1) //main work loop
			{
				// DISPLAY PAGE REFRESH
				if((HAL_GetTick() - timerLcdInput) > 200) //if timer or input changed
				{
					if(changePage)
					{
						currentLcdPage++;
						if(currentLcdPage == LCD_NOPAGE) currentLcdPage = LCD_PARAM;
						changePage = 0;
					}
					f_gui_DrawPage(currentLcdPage, &PidParam, &PidCtrl, motorPwm, *distanceSet, distanceGet);

					timerLcdInput = HAL_GetTick();
				}

				// DISPLAY LCD HEADING
				if(( HAL_GetTick() - timerLcdHeading) > 500)
				{
					f_gui_DrawHeading(SM.currentState, currentLcdPage);
					timerLcdHeading = HAL_GetTick();
				}

				// REFRESH DISTANCE SET
				if(encoderInputChanged) //if encoder input changed
				{
					if(*distanceSet > PINGPONG_MAX_DISTANCE) *distanceSet = PINGPONG_MAX_DISTANCE;
					else if(*distanceSet < PINGPONG_MIN_DISTANCE) *distanceSet = PINGPONG_MIN_DISTANCE;
					encoderInputChanged = false;

					if(currentLcdPage == LCD_CTRL) f_gui_DrawPage(currentLcdPage, &PidParam, &PidCtrl, motorPwm, *distanceSet, distanceGet); //to prevent to big refresh latency
				}

				// MOTOR PWM SET
				if((HAL_GetTick() - timerMotor) > 20)
				{
					motorPwm = f_runtime_WorkMotorHandler(&distanceGet, distanceSet, &motorPwm, &PidParam, &PidCtrl);

					motorPwmMean = (15*motorPwmMean + motorPwm)/16;
					timerMotor = HAL_GetTick();
				}

				// MOTOR AND SENSOR TEST
				if((HAL_GetTick() - timerMotorTest) > 200)
				{
					if(abs(motorPwmMean - motorPwm) < (MAX_MOTOR_PWM/100)) // if motorPwm is stabilized
					{
						if((distanceGet == 0) || !f_machine_MotorTestIfOk())
						{
							f_machine_MotorSet(0);
							nextState = ST_ERROR;
							break;
						}
					}
					timerMotorTest = HAL_GetTick();
				}

				HAL_IWDG_Refresh(&hiwdg);

				if(eventFlag) //check if anything happened
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
			} //end of main work loop
			break;

		case POSTPARE:
			f_machine_MotorSet(0);
			break;

		default:
			break;
	}

	Substate++;
	f_sm_ClearEventFlag();
	if(Substate == EXIT) nextState = ST_REPLAY;

	return nextState;
}

e_sm_State f_sm_Replay()
{
	e_sm_State nextState;

	f_lcd_ClearAll();
	f_gui_DrawHeading(SM.currentState, LCD_NOPAGE);
	f_lcd_WriteTxt(0, 32, "Replay?", &font_msSansSerif_14);

	while(!eventFlag)
		;

	if(SM.event == EV_BUTTON_A)	nextState = ST_IDLE;
	else if(SM.event == EV_BUTTON_B) nextState = ST_EXIT;
	else if(SM.event == EV_ERROR) nextState = ST_ERROR;
	f_sm_ClearEventFlag();

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

//================= NO STATE MACHINE FUNCTIONS ===============

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

// =========== INTERRUPT CALLBACKS ===================

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if((GPIO_Pin == B_NEXT_Pin) && HAL_GPIO_ReadPin(B_NEXT_GPIO_Port, B_NEXT_Pin))
	{
		Button.counterEnableA = true;
		Button.counterA = HAL_GetTick();
	}
	else if((GPIO_Pin == B_PREV_Pin) && HAL_GPIO_ReadPin(B_PREV_GPIO_Port, B_PREV_Pin))
	{
		Button.counterEnableB = true;
		Button.counterB = HAL_GetTick();
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	f_machine_SensorOutputCompareCallback(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	f_machine_SensorPeriodElapsedCallback(htim);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	f_machine_SensorCaptureCallback(htim);
	if(htim->Instance == TIM3) encoderInputChanged = true;
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
