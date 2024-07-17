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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Daemon */
osThreadId_t DaemonHandle;
const osThreadAttr_t Daemon_attributes = {
  .name = "Daemon",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for Test */
osThreadId_t TestHandle;
const osThreadAttr_t Test_attributes = {
  .name = "Test",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for gimbal */
osThreadId_t gimbalHandle;
const osThreadAttr_t gimbal_attributes = {
  .name = "gimbal",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis */
osThreadId_t chassisHandle;
const osThreadAttr_t chassis_attributes = {
  .name = "chassis",
  .stack_size = 258 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for arm */
osThreadId_t armHandle;
const osThreadAttr_t arm_attributes = {
  .name = "arm",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cmd */
osThreadId_t cmdHandle;
const osThreadAttr_t cmd_attributes = {
  .name = "cmd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for airpump */
osThreadId_t airpumpHandle;
const osThreadAttr_t airpump_attributes = {
  .name = "airpump",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for referee */
osThreadId_t refereeHandle;
const osThreadAttr_t referee_attributes = {
  .name = "referee",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void _BuzzerTask(void *argument);
void _DaemonTask(void *argument);
void TestTask(void *argument);
void _gimbalTASK(void *argument);
void _chassisTASK(void *argument);
void _armTASK(void *argument);
void _cmdTASK(void *argument);
void _motorTASK(void *argument);
void _airpumpTASK(void *argument);
void _refereeTask(void *argument);

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(_BuzzerTask, NULL, &Buzzer_attributes);

  /* creation of Daemon */
  DaemonHandle = osThreadNew(_DaemonTask, NULL, &Daemon_attributes);

  /* creation of Test */
  TestHandle = osThreadNew(TestTask, NULL, &Test_attributes);

  /* creation of gimbal */
  gimbalHandle = osThreadNew(_gimbalTASK, NULL, &gimbal_attributes);

  /* creation of chassis */
  chassisHandle = osThreadNew(_chassisTASK, NULL, &chassis_attributes);

  /* creation of arm */
  armHandle = osThreadNew(_armTASK, NULL, &arm_attributes);

  /* creation of cmd */
  cmdHandle = osThreadNew(_cmdTASK, NULL, &cmd_attributes);

  /* creation of motor */
  motorHandle = osThreadNew(_motorTASK, NULL, &motor_attributes);

  /* creation of airpump */
  airpumpHandle = osThreadNew(_airpumpTASK, NULL, &airpump_attributes);

  /* creation of referee */
  refereeHandle = osThreadNew(_refereeTask, NULL, &referee_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  UNUSED(argument);
  while(1){
    osDelay(1000);
  }
  // osThreadTerminate(defaultTaskHandle); // 避免空置和切换占用cpu
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header__BuzzerTask */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__BuzzerTask */
__weak void _BuzzerTask(void *argument)
{
  /* USER CODE BEGIN _BuzzerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _BuzzerTask */
}

/* USER CODE BEGIN Header__DaemonTask */
/**
* @brief Function implementing the Daemon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__DaemonTask */
__weak void _DaemonTask(void *argument)
{
  /* USER CODE BEGIN _DaemonTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _DaemonTask */
}

/* USER CODE BEGIN Header_TestTask */
/**
* @brief Function implementing the Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TestTask */
__weak void TestTask(void *argument)
{
  /* USER CODE BEGIN TestTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TestTask */
}

/* USER CODE BEGIN Header__gimbalTASK */
/**
* @brief Function implementing the gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__gimbalTASK */
__weak void _gimbalTASK(void *argument)
{
  /* USER CODE BEGIN _gimbalTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _gimbalTASK */
}

/* USER CODE BEGIN Header__chassisTASK */
/**
* @brief Function implementing the chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__chassisTASK */
__weak void _chassisTASK(void *argument)
{
  /* USER CODE BEGIN _chassisTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _chassisTASK */
}

/* USER CODE BEGIN Header__armTASK */
/**
* @brief Function implementing the arm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__armTASK */
__weak void _armTASK(void *argument)
{
  /* USER CODE BEGIN _armTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _armTASK */
}

/* USER CODE BEGIN Header__cmdTASK */
/**
* @brief Function implementing the cmd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__cmdTASK */
__weak void _cmdTASK(void *argument)
{
  /* USER CODE BEGIN _cmdTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _cmdTASK */
}

/* USER CODE BEGIN Header__motorTASK */
/**
* @brief Function implementing the motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__motorTASK */
__weak void _motorTASK(void *argument)
{
  /* USER CODE BEGIN _motorTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _motorTASK */
}

/* USER CODE BEGIN Header__airpumpTASK */
/**
* @brief Function implementing the airpump thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__airpumpTASK */
__weak void _airpumpTASK(void *argument)
{
  /* USER CODE BEGIN _airpumpTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _airpumpTASK */
}

/* USER CODE BEGIN Header__refereeTask */
/**
* @brief Function implementing the referee thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__refereeTask */
__weak void _refereeTask(void *argument)
{
  /* USER CODE BEGIN _refereeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _refereeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

