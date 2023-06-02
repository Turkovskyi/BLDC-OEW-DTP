/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Inverter 1
#define		SET_SD_A_1			HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_SET)
#define		RESET_SD_A_1		HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_RESET)
#define		SET_SD_B_1			HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_SET)
#define		RESET_SD_B_1		HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_RESET)
#define		SET_SD_C_1			HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_SET)
#define		RESET_SD_C_1		HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_RESET)

#define		IN_A_1				TIM1->CCR1
#define		IN_B_1				TIM1->CCR2
#define		IN_C_1				TIM1->CCR3


//Inverter 2
#define		SET_SD_A_2			HAL_GPIO_WritePin(GPIOB, SD_A_2_Pin, GPIO_PIN_SET)
#define		RESET_SD_A_2		HAL_GPIO_WritePin(GPIOB, SD_A_2_Pin, GPIO_PIN_RESET)
#define		SET_SD_B_2			HAL_GPIO_WritePin(GPIOB, SD_B_2_Pin, GPIO_PIN_SET)
#define		RESET_SD_B_2		HAL_GPIO_WritePin(GPIOB, SD_B_2_Pin, GPIO_PIN_RESET)
#define		SET_SD_C_2			HAL_GPIO_WritePin(GPIOB, SD_C_2_Pin, GPIO_PIN_SET)
#define		RESET_SD_C_2		HAL_GPIO_WritePin(GPIOB, SD_C_2_Pin, GPIO_PIN_RESET)

#define		IN_A_2				TIM8->CCR1
#define		IN_B_2				TIM8->CCR2
#define		IN_C_2				TIM8->CCR3


//Inverter 3
#define		SET_SD_A_3			HAL_GPIO_WritePin(GPIOA, SD_A_3_Pin, GPIO_PIN_SET)
#define		RESET_SD_A_3		HAL_GPIO_WritePin(GPIOA, SD_A_3_Pin, GPIO_PIN_RESET)
#define		SET_SD_B_3			HAL_GPIO_WritePin(GPIOA, SD_B_3_Pin, GPIO_PIN_SET)
#define		RESET_SD_B_3		HAL_GPIO_WritePin(GPIOA, SD_B_3_Pin, GPIO_PIN_RESET)
#define		SET_SD_C_3			HAL_GPIO_WritePin(GPIOA, SD_C_3_Pin, GPIO_PIN_SET)
#define		RESET_SD_C_3		HAL_GPIO_WritePin(GPIOA, SD_C_3_Pin, GPIO_PIN_RESET)

#define		IN_A_3				TIM2->CCR1
#define		IN_B_3				TIM2->CCR2
#define		IN_C_3				TIM2->CCR3


//Inverter 4
#define		SET_SD_A_4			HAL_GPIO_WritePin(GPIOA, SD_A_4_Pin, GPIO_PIN_SET)
#define		RESET_SD_A_4		HAL_GPIO_WritePin(GPIOA, SD_A_4_Pin, GPIO_PIN_RESET)
#define		SET_SD_B_4			HAL_GPIO_WritePin(GPIOA, SD_B_4_Pin, GPIO_PIN_SET)
#define		RESET_SD_B_4		HAL_GPIO_WritePin(GPIOA, SD_B_4_Pin, GPIO_PIN_RESET)
#define		SET_SD_C_4			HAL_GPIO_WritePin(GPIOA, SD_C_4_Pin, GPIO_PIN_SET)
#define		RESET_SD_C_4		HAL_GPIO_WritePin(GPIOA, SD_C_4_Pin, GPIO_PIN_RESET)

#define		IN_A_4				TIM1->CCR4
#define		IN_B_4				TIM8->CCR4
#define		IN_C_4				TIM2->CCR4

// Define constants
const int MAX_PWM_VALUE = 4095;
const int PWM_INCREMENT = 25;
const int PWM_DECREMENT = -25;
const int STATE_0_DURATION = 1500;
const int STATE_OFF_DURATION = 2500;
const int TIMER_REPEAT = 20;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */


uint16_t PWM = 2048;



volatile uint16_t PWM1 = 0;
uint16_t PWM2 = 0;
uint16_t PWM3 = 0;
uint16_t PWM4 = 0;

uint16_t Delay = 6;

char debug_UART1[6] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void state0_inv1(void);
void state1_inv1(void);
void state2_inv1(void);
void state3_inv1(void);
void state4_inv1(void);
void state5_inv1(void);
void state6_inv1(void);
void stateOff_inv1(void);

void state0_inv2(void);
void state1_inv2(void);
void state2_inv2(void);
void state3_inv2(void);
void state4_inv2(void);
void state5_inv2(void);
void state6_inv2(void);
void stateOff_inv2(void);


void state0_inv3(void);
void state1_inv3(void);
void state2_inv3(void);
void state3_inv3(void);
void state4_inv3(void);
void state5_inv3(void);
void state6_inv3(void);
void stateOff_inv3(void);


void state0_inv4(void);
void state1_inv4(void);
void state2_inv4(void);
void state3_inv4(void);
void state4_inv4(void);
void state5_inv4(void);
void state6_inv4(void);
void stateOff_inv4(void);

void Module1_2inv(void);
void Module2_2inv(void);

void Inverter1(uint8_t);
void Inverter2(uint8_t);
void Inverter3(uint8_t);
void Inverter4(uint8_t);


// Define function to set inverter states
void setInverterStates(int state1, int state2, int state3, int state4) {
  Inverter1(state1);
  Inverter2(state2);
  Inverter3(state3);
  Inverter4(state4);
}

// Define function to check if a certain amount of time has passed since a state transition
uint8_t hasStateTransitionElapsed(int prevStateIndex, int currentTime, int stateOffTime[]) {
  return (currentTime - stateOffTime[prevStateIndex]) >= STATE_OFF_DURATION;
}

/*
InverterX(1) - all switches are off
InverterX(2) - low switches are on, high switches are off
InverterX(3) - 120 deg. commutation with PWM
InverterX(4) - 120 deg. commutation without PWM
*/

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
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);


  PWM1 = 0;
  PWM2 = 0;
  PWM3 = 0;
  PWM4 = 0;

//  TIM1->CCR1 = 2048;
//  TIM1->CCR2 = 2048;
//  TIM1->CCR3 = 2048;
//  TIM1->CCR4 = 1048;
//
//  TIM2->CCR1 = 2048;
//  TIM2->CCR2 = 2048;
//  TIM2->CCR3 = 2048;
//  TIM2->CCR4 = 2048;
//
//  TIM8->CCR1 = 2048;
//  TIM8->CCR2 = 2048;
//  TIM8->CCR3 = 2048;
//  TIM8->CCR4 = 2048;
  uint32_t timer;
  uint32_t time;

  uint8_t currentState = 0;
  uint32_t stateOffTime[20] = {0};
//  uint32_t timer1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		//new line
//		Module1_2inv();
//		Module2_2inv();

		/*
		InverterX(1) - all switches are off
		InverterX(2) - low switches are on, high switches are off
		InverterX(3) - 120 deg. commutation with PWM
		InverterX(4) - 120 deg. commutation without PWM
		*/

		time = HAL_GetTick();

		switch (currentState)
		{
		case 0:

		setInverterStates(1, 1, 1, 1);

		if (time >= STATE_0_DURATION) {
		stateOffTime[0] = time;
		currentState = 1;
		}
		break;


			break;
		case 1:
			if (time >= timer)
			{
				timer = time + TIMER_REPEAT;
				PWM1 += PWM_INCREMENT;
				PWM2 += PWM_INCREMENT;
			}

			if (PWM1 > MAX_PWM_VALUE)
			{
				PWM1 = MAX_PWM_VALUE;
				PWM2 = MAX_PWM_VALUE;
				currentState = 2;
				stateOffTime[1] = time;
			}
			setInverterStates(3, 3, 2, 2);
			break;

		case 2:
		      if (hasStateTransitionElapsed(1, time, stateOffTime))
		      {
				currentState = 3;
				stateOffTime[2] = time;
			}
		      setInverterStates(4, 1, 2, 1);
			break;

		case 3:
			if (time >= timer)
			{
				timer = time + TIMER_REPEAT;
				PWM3 += PWM_INCREMENT;
				PWM4 += PWM_INCREMENT;
			}

			if (PWM3 > MAX_PWM_VALUE) {
				PWM3 = MAX_PWM_VALUE;
				PWM4 = MAX_PWM_VALUE;
				currentState = 4;
				stateOffTime[3] = time;
			}

		      setInverterStates(4, 4, 3, 3);

			break;

		case 4:
		  if (hasStateTransitionElapsed(3, time, stateOffTime))
		{
			currentState = 5;
			stateOffTime[2] = time;
		}

	      setInverterStates(4, 1, 4, 1);
		break;

		case 5:
			if (time >= timer)
			{
				timer = time + TIMER_REPEAT;
				PWM3 += PWM_DECREMENT;
				PWM4 += PWM_DECREMENT;
			}

			if (PWM3 < PWM_INCREMENT)
			{
				PWM3 = 0;
				PWM4 = 0;
				currentState = 6;
				stateOffTime[5] = time;
			}
			setInverterStates(4, 1, 3, 1);
			break;

		case 6:
			if (hasStateTransitionElapsed(5, time, stateOffTime))
			{
				currentState = 7;
				stateOffTime[6] = time;
			}
			setInverterStates(4, 1, 2, 1);
			break;

		case 7:
			if (time >= timer)
			{
				timer = time + 20;
				PWM1 += PWM_DECREMENT;
				PWM2 += PWM_DECREMENT;
			}

			if (PWM1 < 100)
			{
				PWM1 = 0;
				PWM2 = 0;
				currentState = 8;
				stateOffTime[7] = time;
			}
			setInverterStates(3, 1, 2, 1);
			break;

		case 8:
			setInverterStates(1, 1, 1, 1);






//		case 3:
//			if (time >= timer)
//			{
//				timer = time + 20;
//				PWM1 = PWM1 - 10;
//				PWM2 -= 10;
//			}
//
//			if (PWM1 < 20)
//			{
//				PWM1 = 0;
//				PWM2 = 0;
//				currentState = 0;
//				state3OffTime = time;
//			}
////			PWM1 = 2000;
////			PWM2 = 2000;
//
//			Inverter1(3);
//			Inverter2(3);
//			Inverter3(2);
//			Inverter4(2);
//			break;
		}




/*
		if (time < 15000) {
			Inverter1(3);
			Inverter2(1);
			Inverter3(2);
			Inverter4(1);
			if (time >= timer) {
				timer += 20;
				PWM1 = PWM1 + 10;
				PWM2 += 10;

				if (PWM1 > 4095)
					PWM1 = 4095;
				if (PWM2 > 4095)
					PWM2 = 4095;
			}
		}
		else if (time < 30000) {
			Inverter1(4);
			Inverter2(1);
			Inverter3(3);
			if (time >= timer) {
				timer += 20;
				PWM3 = PWM3 + 10;
				PWM4 += 10;

				if (PWM3 > 4095)
					PWM3 = 4095;
				if (PWM4 > 4095)
					PWM4 = 4095;

			}
		}
		else {
			Inverter1(4);
			Inverter2(1);
			Inverter3(3);
		}

		*/
//		else if (time < 40000) {
//			Inverter1(4);
//			Inverter2(1);
//			Inverter3(3);
//			if (time >= timer) {
//				timer += 20;
//				PWM3 = PWM3 - 10;
//				PWM4 += 10;
//
//				if (PWM3 < 40)
//					PWM3 = 0;
//				if (PWM4 < 40)
//					PWM4 = 0;
//			}
//		}




//		if (HAL_GetTick() >= timer) {
//			timer += 7;
//			Delay++;
//
//			if (Delay >=7) Delay = 1;
//
//			if (Delay == 1) state1_inv1();
//
//			else if (Delay == 2) state2_inv1();
//
//			else if (Delay == 3) state3_inv1();
//
//			else if (Delay == 4) state4_inv1();
//
//			else if (Delay == 5) state5_inv1();
//
//			else if (Delay == 6) state6_inv1();
//		}


//		state1_inv1();
//		HAL_Delay(Delay);
//
//		sprintf(debug_UART1, "%d \0\n", (int) HAL_GetTick());
//		HAL_UART_Transmit_IT(&huart1, (uint8_t*) &debug_UART1, 6);
//

//		state1_inv1();
//		HAL_Delay(Delay);
//
//		state2_inv1();
//		HAL_Delay(Delay);
//
//		state3_inv1();
//		HAL_Delay(Delay);
//
//		state4_inv1();
//		HAL_Delay(Delay);
//
//		state5_inv1();
//		HAL_Delay(Delay);
//
//		state6_inv1();
//		HAL_Delay(Delay);


//		state1_inv2();
//		HAL_Delay(Delay);
//
//		state2_inv2();
//		HAL_Delay(Delay);
//
//		state3_inv2();
//		HAL_Delay(Delay);
//
//		state4_inv2();
//		HAL_Delay(Delay);
//
//		state5_inv2();
//		HAL_Delay(Delay);
//
//		state6_inv2();
//		HAL_Delay(Delay);



//	   state6_inv1();
//	   state6_inv3();
//
//	   state6_inv2();
//	   state6_inv4();
//
//	   HAL_Delay(Delay);
//
//
//	   state5_inv1();
//	   state5_inv3();
//
//	   state5_inv2();
//	   state5_inv4();
//
//	   HAL_Delay(Delay);
//
//	   state4_inv1();
//	   state4_inv3();
//
//	   state4_inv2();
//	   state4_inv4();
//
//	   HAL_Delay(Delay);
//
//	   state3_inv1();
//	   state3_inv3();
//
//	   state3_inv2();
//	   state3_inv4();
//
//	   HAL_Delay(Delay);
//
//	   state2_inv1();
//	   state2_inv3();
//
//	   state2_inv2();
//	   state2_inv4();
//
//	   HAL_Delay(Delay);
//
//	   state1_inv1();
//	   state1_inv3();
//
//	   state1_inv2();
//	   state1_inv4();
//
//	   HAL_Delay(Delay);







//	  if(HAL_GPIO_ReadPin(GPIOC, HS4_Pin))
//			  HAL_GPIO_WritePin(GPIOB, SD_A_2_Pin, GPIO_PIN_SET);
//	  else
//		  HAL_GPIO_WritePin(GPIOB, SD_A_2_Pin, GPIO_PIN_RESET);
//
//	  if(HAL_GPIO_ReadPin(GPIOC, HS5_Pin))
//	  			  HAL_GPIO_WritePin(GPIOB, SD_B_2_Pin, GPIO_PIN_SET);
//	  else
//		  HAL_GPIO_WritePin(GPIOB, SD_B_2_Pin, GPIO_PIN_RESET);
//
//	  if(HAL_GPIO_ReadPin(GPIOC, HS6_Pin))
//	  			  HAL_GPIO_WritePin(GPIOB, SD_C_2_Pin, GPIO_PIN_SET);
//	  else
//		  HAL_GPIO_WritePin(GPIOB, SD_C_2_Pin, GPIO_PIN_RESET);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 4095;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 4095;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_A_3_Pin|SD_B_3_Pin|SD_C_3_Pin|SD_A_4_Pin
                          |SD_B_4_Pin|SD_C_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin|SD_C_1_Pin|SD_A_1_Pin|Toggle_pin_Pin
                          |SD_A_2_Pin|SD_B_2_Pin|SD_C_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_A_3_Pin SD_B_3_Pin SD_C_3_Pin SD_A_4_Pin
                           SD_B_4_Pin SD_C_4_Pin */
  GPIO_InitStruct.Pin = SD_A_3_Pin|SD_B_3_Pin|SD_C_3_Pin|SD_A_4_Pin
                          |SD_B_4_Pin|SD_C_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_B_1_Pin SD_C_1_Pin SD_A_1_Pin Toggle_pin_Pin
                           SD_A_2_Pin SD_B_2_Pin SD_C_2_Pin */
  GPIO_InitStruct.Pin = SD_B_1_Pin|SD_C_1_Pin|SD_A_1_Pin|Toggle_pin_Pin
                          |SD_A_2_Pin|SD_B_2_Pin|SD_C_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HS4_Pin HS5_Pin HS6_Pin */
  GPIO_InitStruct.Pin = HS4_Pin|HS5_Pin|HS6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HS1_Pin HS2_Pin HS3_Pin */
  GPIO_InitStruct.Pin = HS1_Pin|HS2_Pin|HS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


//void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef * htim){
//HAL_GPIO_TogglePin (GPIOB, Toggle_pin_Pin);
//}

void state0_inv1() {

	IN_A_1 = 0;
	SET_SD_A_1;

	IN_B_1 = 0;
	SET_SD_B_1;

	IN_C_1 = 0;
	SET_SD_C_1;

}

void state1_inv1() {

	IN_A_1 = 0;
	RESET_SD_A_1;

	IN_B_1 = PWM1;
	SET_SD_B_1;

	IN_C_1 = 0;
	SET_SD_C_1;



//		TIM1->CCR1 = 0;
//		TIM1->CCR2 = PWM1;
//		TIM1->CCR3 = PWM1;
//    	HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_RESET);
}
void state2_inv1() {

	IN_A_1 = PWM1;
	SET_SD_A_1;

	IN_B_1 = 0;
	RESET_SD_B_1;

	IN_C_1 = 0;
	SET_SD_C_1;

//		TIM1->CCR1 = PWM1;
//		TIM1->CCR2 = 0;
//		TIM1->CCR3 = PWM1;
//    	HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_RESET);
}

void state3_inv1() {

	IN_A_1 = PWM1;
	SET_SD_A_1;

	IN_B_1 = 0;
	SET_SD_B_1;

	IN_C_1 = 0;
	RESET_SD_C_1;

//		TIM1->CCR1 = PWM1;
//		TIM1->CCR2 = PWM1;
//		TIM1->CCR3 = 0;
//    	HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_RESET);
}

void state4_inv1() {

	IN_A_1 = 0;
	RESET_SD_A_1;

	IN_B_1 = 0;
	SET_SD_B_1;

	IN_C_1 = PWM1;
	SET_SD_C_1;

//		TIM1->CCR1 = 0;
//		TIM1->CCR2 = PWM1;
//		TIM1->CCR3 = PWM1;
//    	HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_SET);
}

void state5_inv1() {

	IN_A_1 = 0;
	SET_SD_A_1;

	IN_B_1 = 0;
	RESET_SD_B_1;

	IN_C_1 = PWM1;
	SET_SD_C_1;

//	TIM1->CCR1 = PWM1;
//	TIM1->CCR2 = 0;
//	TIM1->CCR3 = PWM1;
//	HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_SET);
}

void state6_inv1() {

	IN_A_1 = 0;
	SET_SD_A_1;

	IN_B_1 = PWM1;
	SET_SD_B_1;

	IN_C_1 = 0;
	RESET_SD_C_1;

//		TIM1->CCR1 = PWM1;
//		TIM1->CCR2 = PWM1;
//		TIM1->CCR3 = 0;
//    	HAL_GPIO_WritePin(GPIOB, SD_A_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, SD_B_1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB, SD_C_1_Pin, GPIO_PIN_RESET);
}

void stateOff_inv1() {

	IN_A_1 = 0;
	RESET_SD_A_1;


	IN_B_1 = 0;
	RESET_SD_B_1;


	IN_C_1 = 0;
	RESET_SD_C_1;

}


//Inverter 2
void state0_inv2() {
	IN_A_2 = 0;
	SET_SD_A_2;

	IN_B_2 = 0;
	SET_SD_B_2;

	IN_C_2 = 0;
	SET_SD_C_2;
}


void state1_inv2() {
	IN_A_2 = 0;
	RESET_SD_A_2;

	IN_B_2 = PWM2;
	SET_SD_B_2;

	IN_C_2 = 0;
	SET_SD_C_2;
}
void state2_inv2() {
	IN_A_2 = PWM2;
	SET_SD_A_2;

	IN_B_2 = 0;
	RESET_SD_B_2;

	IN_C_2 = 0;
	SET_SD_C_2;
}
void state3_inv2() {
	IN_A_2 = PWM2;
	SET_SD_A_2;

	IN_B_2 = 0;
	SET_SD_B_2;

	IN_C_2 = 0;
	RESET_SD_C_2;
}
void state4_inv2() {
	IN_A_2 = 0;
	RESET_SD_A_2;

	IN_B_2 = 0;
	SET_SD_B_2;

	IN_C_2 = PWM2;
	SET_SD_C_2;
}
void state5_inv2() {
	IN_A_2 = 0;
	SET_SD_A_2;

	IN_B_2 = 0;
	RESET_SD_B_2;

	IN_C_2 = PWM2;
	SET_SD_C_2;
}
void state6_inv2() {
	IN_A_2 = 0;
	SET_SD_A_2;

	IN_B_2 = PWM2;
	SET_SD_B_2;

	IN_C_2 = 0;
	RESET_SD_C_2;
}

void stateOff_inv2() {
	IN_A_2 = 0;
	RESET_SD_A_2;

	IN_B_2 = 0;
	RESET_SD_B_2;

	IN_C_2 = 0;
	RESET_SD_C_2;
}

//Inverter 3
void state0_inv3() {
	IN_A_3 = 0;
	SET_SD_A_3;

	IN_B_3 = 0;
	SET_SD_B_3;

	IN_C_3 = 0;
	SET_SD_C_3;
}

void state1_inv3() {
	IN_A_3 = 0;
	RESET_SD_A_3;

	IN_B_3 = 0;
	SET_SD_B_3;

	IN_C_3 = PWM3;
	SET_SD_C_3;
}
void state2_inv3() {
	IN_A_3 = 0;
	SET_SD_A_3;

	IN_B_3 = 0;
	RESET_SD_B_3;

	IN_C_3 = PWM3;
	SET_SD_C_3;
}
void state3_inv3() {
	IN_A_3 = 0;
	SET_SD_A_3;

	IN_B_3 = PWM3;
	SET_SD_B_3;

	IN_C_3 = 0;
	RESET_SD_C_3;
}
void state4_inv3() {
	IN_A_3 = 0;
	RESET_SD_A_3;

	IN_B_3 = PWM3;
	SET_SD_B_3;

	IN_C_3 = 0;
	SET_SD_C_3;
}
void state5_inv3() {
	IN_A_3 = PWM3;
	SET_SD_A_3;

	IN_B_3 = 0;
	RESET_SD_B_3;

	IN_C_3 = 0;
	SET_SD_C_3;
}
void state6_inv3() {
	IN_A_3 = PWM3;
	SET_SD_A_3;

	IN_B_3 = 0;
	SET_SD_B_3;

	IN_C_3 = 0;
	RESET_SD_C_3;
}

void stateOff_inv3() {
	IN_A_3 = 0;
	RESET_SD_A_3;

	IN_B_3 = 0;
	RESET_SD_B_3;

	IN_C_3 = 0;
	RESET_SD_C_3;
}


	//Inverter 4
void state0_inv4() {
	IN_A_4 = 0;
	SET_SD_A_4;

	IN_B_4 = 0;
	SET_SD_B_4;

	IN_C_4 = 0;
	SET_SD_C_4;
}

void state1_inv4() {
	IN_A_4 = 0;
	RESET_SD_A_4;

	IN_B_4 = 0;
	SET_SD_B_4;

	IN_C_4 = PWM4;
	SET_SD_C_4;
}
void state2_inv4() {
	IN_A_4 = 0;
	SET_SD_A_4;

	IN_B_4 = 0;
	RESET_SD_B_4;

	IN_C_4 = PWM4;
	SET_SD_C_4;
}
void state3_inv4() {
	IN_A_4 = 0;
	SET_SD_A_4;

	IN_B_4 = PWM4;
	SET_SD_B_4;

	IN_C_4 = 0;
	RESET_SD_C_4;
}
void state4_inv4() {
	IN_A_4 = 0;
	RESET_SD_A_4;

	IN_B_4 = PWM4;
	SET_SD_B_4;

	IN_C_4 = 0;
	SET_SD_C_4;
}
void state5_inv4() {
	IN_A_4 = PWM4;
	SET_SD_A_4;

	IN_B_4 = 0;
	RESET_SD_B_4;

	IN_C_4 = 0;
	SET_SD_C_4;
}
void state6_inv4() {

	IN_A_4 = PWM4;
	SET_SD_A_4;

	IN_B_4 = 0;
	SET_SD_B_4;

	IN_C_4 = 0;
	RESET_SD_C_4;
}

void stateOff_inv4() {

	IN_A_4 = 0;
	RESET_SD_A_4;

	IN_B_4 = 0;
	RESET_SD_B_4;

	IN_C_4 = 0;
	RESET_SD_C_4;
}

void Module1_2inv(void) {
	if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
			&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
			&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
		state1_inv1();
		state1_inv3();
	}

	else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
			&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
			&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
		state2_inv1();
		state2_inv3();
	}

	else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
			&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
			&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
		state3_inv1();
		state3_inv3();
	}

	else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
			&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
			&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
		state4_inv1();
		state4_inv3();
	}

	else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
			&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
			&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
		state5_inv1();
		state5_inv3();
	}

	else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
			&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
			&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
		state6_inv1();
		state6_inv3();
	}

}
void Module2_2inv(void) {
	if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
			&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
			&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
		state1_inv2();
		state1_inv4();
	}

	else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
			&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
			&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
		state2_inv2();
		state2_inv4();
	}

	else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
			&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
			&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
		state3_inv2();
		state3_inv4();
	}

	else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
			&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
			&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
		state4_inv2();
		state4_inv4();
	}

	else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
			&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
			&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
		state5_inv2();
		state5_inv4();
	}

	else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
			&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
			&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
		state6_inv2();
		state6_inv4();
	}
}

void Inverter1(uint8_t inv_mode) {
	/*
	inv_mode == 1- all switches are off
	inv_mode == 2- low switches are on, high switches are off
	inv_mode == 3- 120 deg. commutation with PWM
	inv_mode == 4- 120 deg. commutation without PWM
	*/

	if (inv_mode == 1) {
		stateOff_inv1();
	}

	else if (inv_mode == 2) {
		state0_inv1();
	}
	else if (inv_mode == 3) {
		if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin) && HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state1_inv1();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state2_inv1();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state3_inv1();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state4_inv1();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state5_inv1();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state6_inv1();
		}
	}
		else if (inv_mode == 4) {
			PWM1= 4095;
		if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin) && HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state1_inv1();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state2_inv1();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state3_inv1();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state4_inv1();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state5_inv1();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state6_inv1();
		}
	}
}
void Inverter3(uint8_t inv_mode) {
	if (inv_mode == 1) {
		stateOff_inv3();
	}

	else if (inv_mode == 2) {
		state0_inv3();
	}
	else if (inv_mode == 3) {
		if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin) && HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state1_inv3();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state2_inv3();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state3_inv3();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state4_inv3();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state5_inv3();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state6_inv3();
		}
	}

		else if (inv_mode == 4) {
			PWM3 = 4095;
		if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin) && HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state1_inv3();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state2_inv3();
		}

		else if (HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state3_inv3();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state4_inv3();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state5_inv3();
		}

		else if (!HAL_GPIO_ReadPin(GPIOB, HS1_Pin)
				&& HAL_GPIO_ReadPin(GPIOB, HS2_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, HS3_Pin)) {
			state6_inv3();
		}
	}
}
void Inverter2(uint8_t inv_mode) {
	if (inv_mode == 1) {
		stateOff_inv2();
	}

	else if (inv_mode == 2) {
		state0_inv2();
	}
	else if (inv_mode == 3) {
		if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state1_inv2();
			}

			else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state2_inv2();
			}

			else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state3_inv2();
			}

			else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state4_inv2();
			}

			else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state5_inv2();
			}

			else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state6_inv2();
			}
	}

	else if (inv_mode == 4) {
		PWM2 = 4095;
		if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin) && HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state1_inv2();
		}

		else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state2_inv2();
		}

		else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state3_inv2();
		}

		else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state4_inv2();
		}

		else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state5_inv2();
		}

		else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state6_inv2();
		}
	}
}
void Inverter4(uint8_t inv_mode) {
	if (inv_mode == 1) {
		stateOff_inv4();
	}

	else if (inv_mode == 2) {
		state0_inv4();
	}
	else if (inv_mode == 3) {
		if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state1_inv4();
			}

			else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state2_inv4();
			}

			else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state3_inv4();
			}

			else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state4_inv4();
			}

			else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state5_inv4();
			}

			else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
					&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
					&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
				state6_inv4();
			}
	}

	else if (inv_mode == 4) {
		PWM4 = 4095;
		if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin) && HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state1_inv4();
		}

		else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state2_inv4();
		}

		else if (HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state3_inv4();
		}

		else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state4_inv4();
		}

		else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state5_inv4();
		}

		else if (!HAL_GPIO_ReadPin(GPIOC, HS4_Pin)
				&& HAL_GPIO_ReadPin(GPIOC, HS5_Pin)
				&& !HAL_GPIO_ReadPin(GPIOC, HS6_Pin)) {
			state6_inv4();
		}
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
