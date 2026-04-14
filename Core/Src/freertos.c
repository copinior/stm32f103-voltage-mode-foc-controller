/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "global_def.h"
#include "motor/motor_runtime_param.h"
#include "motor/motor_foc.h"
#include "motor/motor_control_service.h"
#include "motor/as5600.h"
#include "motor/motor_cmd.h"
#include "app_event.h"
#include "usart.h"
#include "tim.h"
#include "Drivers/voice_bridge/voice_bridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void encoder_sample_update(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for CmdTask */
osThreadId_t CmdTaskHandle;
const osThreadAttr_t CmdTask_attributes = {
  .name = "CmdTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for printFlag */
osEventFlagsId_t printFlagHandle;
osStaticEventGroupDef_t printFlagControlBlock;
const osEventFlagsAttr_t printFlag_attributes = {
  .name = "printFlag",
  .cb_mem = &printFlagControlBlock,
  .cb_size = sizeof(printFlagControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void sensorTask(void *argument);
void cmdTask(void *argument);
void telemetryTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(sensorTask, NULL, &SensorTask_attributes);

  /* creation of CmdTask */
  CmdTaskHandle = osThreadNew(cmdTask, NULL, &CmdTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(telemetryTask, NULL, &TelemetryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of printFlag */
  printFlagHandle = osEventFlagsNew(&printFlag_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  app_event_bind_print_flag(printFlagHandle);
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Start UART interrupt reception after RTOS is running */
  usart1_start_rx_it();

  #if VOICE_BRIDGE_ENABLE
  vb_init();
  usart3_start_rx_it();
  #endif

  /* Start control timer after RTOS objects/tasks are fully created */
  HAL_TIM_Base_Start_IT(&htim3);
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_sensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorTask */
void sensorTask(void *argument)
{
  /* USER CODE BEGIN sensorTask */
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    encoder_sample_update();
  }
  /* USER CODE END sensorTask */
}

/* USER CODE BEGIN Header_cmdTask */
/**
* @brief Function implementing the CmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cmdTask */
void cmdTask(void *argument)
{
  /* USER CODE BEGIN cmdTask */
  /* Infinite loop */
  for(;;)
  {
    osEventFlagsWait(printFlagHandle, APP_EVENT_CMD_RECV, osFlagsWaitAny, osWaitForever);

    motor_cmd_poll();
    
    #if VOICE_BRIDGE_ENABLE
    vb_poll();
    #endif
    
    //osEventFlagsClear(printFlagHandle, APP_EVENT_CMD_RECV);
  }
  /* USER CODE END cmdTask */
}

/* USER CODE BEGIN Header_telemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_telemetryTask */
void telemetryTask(void *argument)
{
  /* USER CODE BEGIN telemetryTask */
  /* Infinite loop */
  for(;;)
  {
    osEventFlagsWait(printFlagHandle, APP_EVENT_STATUS_PRINT, osFlagsWaitAny, osWaitForever);
    motor_control_service_print_status();
  }
  /* USER CODE END telemetryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void encoder_sample_update(void)
{
  if (encoder_sample_req) {
      __disable_irq();
      encoder_sample_req = 0;
      __enable_irq();

      uint16_t raw = as5600_read_angle();
      if (raw != 0xFFFF) {
          float raw_angle = (float)raw * 2.0f * PI / 4096.0f;
          float new_angle = raw_angle;

          if (new_angle < 0.0f) new_angle += 2.0f * PI;
          if (new_angle >= 2.0f * PI) new_angle -= 2.0f * PI;

          __disable_irq();
          encoder_angle_cache = new_angle;
          encoder_sample_ready = 1;
          __enable_irq();
      }
  }
}

/* USER CODE END Application */

