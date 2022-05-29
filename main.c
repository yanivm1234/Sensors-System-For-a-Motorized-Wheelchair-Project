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
#include "app_tof.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bno055_stm32.h"
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
 I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//defines:

///////////////pins defiens:
//sonnar1 output trig PA_9 (D8) back
#define SONAR1_TRIG_PORT GPIOA
#define SONAR1_TRIG_PIN GPIO_PIN_9
//sonnar1 input echo PC_7 (D9)
#define SONAR1_ECHO_PORT GPIOC
#define SONAR1_ECHO_PIN GPIO_PIN_7

//sonnar2 output trig PA_6 (D12) front right
#define SONAR2_TRIG_PORT GPIOA
#define SONAR2_TRIG_PIN GPIO_PIN_6
//sonnar2 input echo PA_7 (D11)
#define SONAR2_ECHO_PORT GPIOA
#define SONAR2_ECHO_PIN GPIO_PIN_7

//sonnar3 output trig PB_10 (D6) front left
#define SONAR3_TRIG_PORT GPIOB
#define SONAR3_TRIG_PIN GPIO_PIN_10
//sonnar3 input echo PB_6 (D10)
#define SONAR3_ECHO_PORT GPIOB
#define SONAR3_ECHO_PIN GPIO_PIN_6

//sonnar4 output trig PA_10 (D2) front left
#define SONAR4_TRIG_PORT GPIOA
#define SONAR4_TRIG_PIN GPIO_PIN_10
//sonnar4 input echo PB_0 (A3)
#define SONAR4_ECHO_PORT GPIOB
#define SONAR4_ECHO_PIN GPIO_PIN_0

//alarm system
//button - input PA_0 (A0)
#define BUTTON_PORT GPIOA
#define BUTTON_PIN GPIO_PIN_0
//Led define PA_5 (D13):
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

//Relay
// output PB_5 (D4)
#define relay_PORT1 GPIOB
#define relay_PIN1 GPIO_PIN_5
#define relay_PORT2 GPIOC
#define relay_PIN2 GPIO_PIN_1
//I2c

//time of flight I2c1 hi2c1
//SCL D15 ; SDA D14

//Accelomter I2C3 hi2c3
//SCL D7 ; SDA D5

///////////////////////////////////////

//////////////defines:
#define SONAR_ALARM_DISTANCE 115
#define SONAR_STOP_DISTANCE 25

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}


void delay_us (uint16_t time) // delay in us
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < time){
	}
}


uint32_t HCSR04_Read (GPIO_TypeDef* TRIG_PORT, uint16_t TRIG_Pin, GPIO_TypeDef* ECHO_PORT, uint16_t ECHO_Pin)
{
	uint32_t distance = 0;
	uint32_t time = 0;

	//set the trigger
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_Pin, GPIO_PIN_SET);//pull the TRIG pin HIGH
	delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_Pin, GPIO_PIN_RESET);//pull the TRIG pin LOW

    // read the time for which the pin is high
   while (!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_Pin)){
   }
   __HAL_TIM_SET_COUNTER(&htim1,0);
   while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_Pin)){
   }

   //calculate the distance
   time = (uint32_t)__HAL_TIM_GET_COUNTER(&htim1);
   distance = time/58;
   printf("distance [cm] %lu \n", distance);
   return distance;
}

void Start_alarm_sys (){
	printf("start alarm system \n");
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // light the led
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // off led
}

void calc_disable_alarm(){
	if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET){
		printf("disable alarm system for 1 min \n");
		HAL_Delay(60000);
	}
}

void stop_motor(){
	HAL_GPIO_WritePin(relay_PORT1, relay_PIN1 , GPIO_PIN_SET); // stop the motor
	HAL_GPIO_WritePin(relay_PORT2, relay_PIN2 , GPIO_PIN_SET); // stop the motor
	printf("stop motor!! \n");
	HAL_Delay(7000);
	HAL_GPIO_WritePin(relay_PORT1, relay_PIN1 , GPIO_PIN_RESET); // RESET THE RELAY
	HAL_GPIO_WritePin(relay_PORT2, relay_PIN2 , GPIO_PIN_RESET); // RESET THE RELAY
}

int Read_sensors_forward(){
	//while for all the sensors
	uint32_t distacne_sensor_1 = 400;
	uint32_t distacne_sensor_4 = 400;
	int flag = 0;
	//	calc sonnar
	distacne_sensor_1 = HCSR04_Read(SONAR1_TRIG_PORT, SONAR1_TRIG_PIN, SONAR1_ECHO_PORT, SONAR1_ECHO_PIN);
	distacne_sensor_4 = HCSR04_Read(SONAR4_TRIG_PORT, SONAR4_TRIG_PIN, SONAR4_ECHO_PORT, SONAR4_ECHO_PIN);
	// calc TOF
	flag = MX_TOF_Process();
	if((distacne_sensor_1 < SONAR_STOP_DISTANCE) || (flag == 2) ||  (distacne_sensor_4 < SONAR_STOP_DISTANCE)){
		stop_motor();
		printf("stop motor!! \n");
		return 0;
	}
	if((distacne_sensor_1 < SONAR_ALARM_DISTANCE) || (flag == 1)||  (distacne_sensor_4 < SONAR_ALARM_DISTANCE)){
		Start_alarm_sys();
		printf("state: 3\n");
		return 3;
	}
	return 2;
}


int Read_sensors_backward(){
	uint32_t distacne_sensor_2 = 400;
	uint32_t distacne_sensor_3 = 400;
	//	calc sonnar
	distacne_sensor_2 = HCSR04_Read(SONAR2_TRIG_PORT, SONAR2_TRIG_PIN, SONAR2_ECHO_PORT, SONAR2_ECHO_PIN);
	distacne_sensor_3 = HCSR04_Read(SONAR3_TRIG_PORT, SONAR3_TRIG_PIN, SONAR3_ECHO_PORT, SONAR3_ECHO_PIN);
	if((distacne_sensor_2 < SONAR_STOP_DISTANCE) || (distacne_sensor_3 < SONAR_STOP_DISTANCE)){
		stop_motor();
		printf("stop motor!! \n");
	return 0;
	}
	if((distacne_sensor_2 < SONAR_ALARM_DISTANCE) || (distacne_sensor_3 < SONAR_ALARM_DISTANCE)){
		Start_alarm_sys();
		printf("state: 3\n");
		return 3;
	}
	return 1;
}

int calc_state(){
	bno055_vector_t mes = bno055_getVectorAccelerometer();
    if (mes.x > 0.75){
    	return 1;
    }
    else if (mes.x < 0.3){
    	return 2;
    }
    else{
    	return 0;
    }
}

int open_sensor(int state){
	 printf("state: %d\n", state);
	 if (state == 1){
		 state = Read_sensors_forward();
	 }
	 else if(state == 2){
		 state = Read_sensors_backward();
	 }
	 HAL_Delay(200);
	 return state;
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
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TOF_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  printf("hello\n");

  //init the accelometer
  bno055_assignI2C(&hi2c3);
  bno055_setup();
  bno055_setOperationModeNDOF();
  HAL_GPIO_WritePin(relay_PORT1, relay_PIN1 , GPIO_PIN_RESET); // RESET THE RELAY
  HAL_GPIO_WritePin(relay_PORT2, relay_PIN2 , GPIO_PIN_RESET); // RESET THE RELAY
  int state = 0;         // 0: stand_by; 1:driving; 2:reverse; 3:activate_alarm


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
	  	  printf("start iteration!\n");

	  	  calc_disable_alarm();
	  	  state = calc_state();

	  	  //open_sensor and calc new state
	  	  state = open_sensor(state);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SONAR2_TRIG_Pin|SONAR1_TRIG_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_Pin SOMAR2_ECHO_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin|SOMAR2_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_XSHUT_Pin LD2_Pin SONAR2_TRIG_Pin SONAR1_TRIG_Pin
                           PA10 */
  GPIO_InitStruct.Pin = TOF_XSHUT_Pin|LD2_Pin|SONAR2_TRIG_Pin|SONAR1_TRIG_Pin
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_INT_Pin */
  GPIO_InitStruct.Pin = TOF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 RELAY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SONAR1_ECHO_Pin */
  GPIO_InitStruct.Pin = SONAR1_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONAR1_ECHO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
