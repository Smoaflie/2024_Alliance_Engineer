/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Daemon */
osThreadId_t DaemonHandle;
const osThreadAttr_t Daemon_attributes = {
  .name = "Daemon",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for robottask */
osThreadId_t robottaskHandle;
const osThreadAttr_t robottask_attributes = {
  .name = "robottask",
  .stack_size = 1028 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for startmotor */
osThreadId_t startmotorHandle;
const osThreadAttr_t startmotor_attributes = {
  .name = "startmotor",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INIT(void *argument);
void BuzzerTask(void *argument);
void DaemonTask(void *argument);
void StartROBOTTASK(void *argument);
void StartMOTORTASK(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  defaultTaskHandle = osThreadNew(INIT, NULL, &defaultTask_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(BuzzerTask, NULL, &Buzzer_attributes);

  /* creation of Daemon */
  DaemonHandle = osThreadNew(DaemonTask, NULL, &Daemon_attributes);

  /* creation of robottask */
  robottaskHandle = osThreadNew(StartROBOTTASK, NULL, &robottask_attributes);

  /* creation of startmotor */
  startmotorHandle = osThreadNew(StartMOTORTASK, NULL, &startmotor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_INIT */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INIT */
__weak void INIT(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INIT */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INIT */
}

/* USER CODE BEGIN Header_BuzzerTask */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BuzzerTask */
__weak void BuzzerTask(void *argument)
{
  /* USER CODE BEGIN BuzzerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BuzzerTask */
}

/* USER CODE BEGIN Header_DaemonTask */
/**
* @brief Function implementing the Daemon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DaemonTask */
__weak void DaemonTask(void *argument)
{
  /* USER CODE BEGIN DaemonTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DaemonTask */
}

/* USER CODE BEGIN Header_StartROBOTTASK */
/**
* @brief Function implementing the robottask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartROBOTTASK */
__weak void StartROBOTTASK(void *argument)
{
  /* USER CODE BEGIN StartROBOTTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartROBOTTASK */
}

/* USER CODE BEGIN Header_StartMOTORTASK */
/**
* @brief Function implementing the startmotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMOTORTASK */
__weak void StartMOTORTASK(void *argument)
{
  /* USER CODE BEGIN StartMOTORTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMOTORTASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

