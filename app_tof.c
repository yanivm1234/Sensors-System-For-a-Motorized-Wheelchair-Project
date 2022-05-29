/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.0.0 instances.
  ******************************************************************************
  *
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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>

#include "custom_ranging_sensor.h"
#include "stm32f4xx_nucleo.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define POLLING_PERIOD (250U) /* milliseconds */

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_Capabilities_t Cap;
static RANGING_SENSOR_ProfileConfig_t Profile;
static int32_t status = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_VL53L1CB_SimpleRanging_Init(void);
int MX_VL53L1CB_SimpleRanging_Process(void);
static void print_result(RANGING_SENSOR_Result_t *Result);
static int32_t decimal_part(float_t x);

void MX_TOF_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */

  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */
  MX_VL53L1CB_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
int MX_TOF_Process(void)
{
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */
	int flag = 0;

	flag = MX_VL53L1CB_SimpleRanging_Process();
	return flag;

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

static void MX_VL53L1CB_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  printf("VL53L1CB Simple Ranging demo application\n");
  status = CUSTOM_RANGING_SENSOR_Init(CUSTOM_VL53L1CB);

  if (status != BSP_ERROR_NONE)
  {
    printf("CUSTOM_RANGING_SENSOR_Init failed\n");
    while(1);
  }
}

int MX_VL53L1CB_SimpleRanging_Process(void)
{
  uint32_t Id;
  RANGING_SENSOR_Result_t Result;
  int flag = 0;
  uint32_t distance = 0;

  CUSTOM_RANGING_SENSOR_ReadID(CUSTOM_VL53L1CB, &Id);
  CUSTOM_RANGING_SENSOR_GetCapabilities(CUSTOM_VL53L1CB, &Cap);

  Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
  Profile.TimingBudget = 30; /* 16 ms < TimingBudget < 500 ms */
  Profile.Frequency = 0; /* not necessary in simple ranging */
  Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  CUSTOM_RANGING_SENSOR_ConfigProfile(CUSTOM_VL53L1CB, &Profile);

  status = CUSTOM_RANGING_SENSOR_Start(CUSTOM_VL53L1CB, RS_MODE_BLOCKING_CONTINUOUS);
  int status_flag = 0;

  while(!status_flag)
  {
     /* polling mode */
     status = CUSTOM_RANGING_SENSOR_GetDistance(CUSTOM_VL53L1CB, &Result);
     if (status == BSP_ERROR_NONE)
     {
    	 distance = (long)Result.ZoneResult[0].Distance[0];
    	 if((Result.ZoneResult[0].Status[0] == 0) && (distance != 0)){
    		 print_result(&Result);
    		 status_flag = 1;
    	     if (distance < 1000){
    	     	flag = 1; //alarm!
    	     }
    	     else if(distance < 200){
    	    	 flag = 2; // stop!
    	     }
    	 }
     }
     HAL_Delay(POLLING_PERIOD);
   }
   status = CUSTOM_RANGING_SENSOR_Stop(CUSTOM_VL53L1CB);
   if (status != BSP_ERROR_NONE){
    printf("CUSTOM_RANGING_SENSOR_Init failed\n");
   }
   return flag;
}

static void print_result(RANGING_SENSOR_Result_t *Result)
{
  uint8_t i, j;

  for (i = 0; i < RANGING_SENSOR_MAX_NB_ZONES; i++)
  {
    printf("\nTargets = %lu", (unsigned long)Result->ZoneResult[i].NumberOfTargets);

    for (j = 0; j < Result->ZoneResult[i].NumberOfTargets; j++)
    {
      printf("\n |---> ");

      printf("Status = %ld, Distance = %5ld mm ",
        (long)Result->ZoneResult[i].Status[j],
        (long)Result->ZoneResult[i].Distance[j]);

      if (Profile.EnableAmbient)
        printf(", Ambient = %ld.%02ld kcps/spad",
          (long)Result->ZoneResult[i].Ambient[j],
          (long)decimal_part(Result->ZoneResult[i].Ambient[j]));

      if (Profile.EnableSignal)
        printf(", Signal = %ld.%02ld kcps/spad",
          (long)Result->ZoneResult[i].Signal[j],
          (long)decimal_part(Result->ZoneResult[i].Signal[j]));
    }
  }
  printf ("\n");
}

static int32_t decimal_part(float_t x)
{
  int32_t int_part = (int32_t) x;
  return (int32_t)((x - int_part) * 100);
}

#ifdef __cplusplus
}
#endif
