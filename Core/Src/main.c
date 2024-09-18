/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "string.h"
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
uint32_t coder;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CANAddFilter(uint32_t rx_id)
{
    CAN_FilterTypeDef can_filter_conf;

    can_filter_conf.FilterMode           = CAN_FILTERMODE_IDLIST; // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale          = CAN_FILTERSCALE_16BIT;,/ // 使用16位id模式,即只有低16位有�?
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO0;          // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.FilterIdLow          = rx_id << 5;            // 过滤器寄存器的低16�?,因为使用STDID,�?以只有低11位有�?,�?5位要�?0
    can_filter_conf.FilterBank           = 1;                     // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation     = CAN_FILTER_ENABLE;     // 启用过滤�?

    HAL_CAN_ConfigFilter(&hcan, &can_filter_conf);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t rx_buf[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxconf, rx_buf); // 从FIFO中获取数�?
    if (rx_buf[0] == 0x11 && rx_buf[1] == 0x88 && rx_buf[2] == 0x99) {
        if (rx_buf[7] == (uint8_t)(rx_buf[3] + rx_buf[4] + rx_buf[5] + rx_buf[6])) {
            memcpy((uint8_t *)&coder, &rx_buf[3], 4);
        }
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    CANAddFilter(0x121);
    coder = 0b0000000001100101101010;

    /* USER CODE BEGIN 2 */
    // uint8_t coder[3];
    // coder[0] = 0xff;
    // coder[1] = 0xff;
    // coder[2] = 0xff;
    // HAL_GPIO_WritePin(X_1_GPIO_Port, X_1_Pin,SET);
    // HAL_GPIO_WritePin(X_1_GPIO_Port, X_7_Pin, SET);
    // HAL_GPIO_WritePin(X_1_GPIO_Port, X_1_Pin, coder[0] & (0x01 << 0));
    // HAL_GPIO_WritePin(X_2_GPIO_Port, X_2_Pin, coder[0] & (0x01 << 1));
    // HAL_GPIO_WritePin(X_3_GPIO_Port, X_3_Pin, coder[0] & (0x01 << 2));
    // HAL_GPIO_WritePin(X_4_GPIO_Port, X_4_Pin, coder[0] & (0x01 << 3));
    // HAL_GPIO_WritePin(X_5_GPIO_Port, X_5_Pin, coder[0] & (0x01 << 4));
    // HAL_GPIO_WritePin(X_6_GPIO_Port, X_6_Pin, coder[0] & (0x01 << 5));
    // HAL_GPIO_WritePin(X_7_GPIO_Port, X_7_Pin, coder[0] & (0x01 << 6));
    // HAL_GPIO_WritePin(X_8_GPIO_Port, X_8_Pin, coder[0] & (0x01 << 7));
    // HAL_GPIO_WritePin(X_9_GPIO_Port, X_9_Pin, coder[1] & (0x01 << 0));
    // HAL_GPIO_WritePin(X_10_GPIO_Port, X_10_Pin, coder[1] & (0x01 << 1));
    // HAL_GPIO_WritePin(X_11_GPIO_Port, X_11_Pin, coder[1] & (0x01 << 2));
    // HAL_GPIO_WritePin(X_12_GPIO_Port, X_12_Pin, coder[1] & (0x01 << 3));
    // HAL_GPIO_WritePin(X_13_GPIO_Port, X_13_Pin, coder[1] & (0x01 << 4));
    // HAL_GPIO_WritePin(X_14_GPIO_Port, X_14_Pin, coder[1] & (0x01 << 5));
    // HAL_GPIO_WritePin(X_15_GPIO_Port, X_15_Pin, coder[1] & (0x01 << 6));
    // HAL_GPIO_WritePin(X_16_GPIO_Port, X_16_Pin, coder[1] & (0x01 << 7));
    // HAL_GPIO_WritePin(X_17_GPIO_Port, X_17_Pin, coder[2] & (0x01 << 0));
    // HAL_GPIO_WritePin(X_18_GPIO_Port, X_18_Pin, coder[2] & (0x01 << 1));
    // HAL_GPIO_WritePin(X_19_GPIO_Port, X_19_Pin, coder[2] & (0x01 << 2));
    // HAL_GPIO_WritePin(X_20_GPIO_Port, X_20_Pin, coder[2] & (0x01 << 3));
    // HAL_GPIO_WritePin(X_21_GPIO_Port, X_21_Pin, coder[2] & (0x01 << 4));
    // HAL_GPIO_WritePin(X_22_GPIO_Port, X_22_Pin, coder[2] & (0x01 << 5));
    // HAL_GPIO_WritePin(X_1_GPIO_Port, X_1_Pin,SET);
    // HAL_GPIO_WritePin(X_1_GPIO_Port, X_7_Pin, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        HAL_GPIO_WritePin(X_1_GPIO_Port, X_1_Pin, coder & (0x00000001 << 0));
        HAL_GPIO_WritePin(X_2_GPIO_Port, X_2_Pin, coder & (0x00000001 << 1));
        HAL_GPIO_WritePin(X_3_GPIO_Port, X_3_Pin, coder & (0x00000001 << 2));
        HAL_GPIO_WritePin(X_4_GPIO_Port, X_4_Pin, coder & (0x00000001 << 3));
        HAL_GPIO_WritePin(X_5_GPIO_Port, X_5_Pin, coder & (0x00000001 << 4));
        HAL_GPIO_WritePin(X_6_GPIO_Port, X_6_Pin, coder & (0x00000001 << 5));
        HAL_GPIO_WritePin(X_7_GPIO_Port, X_7_Pin, coder & (0x00000001 << 6));
        HAL_GPIO_WritePin(X_8_GPIO_Port, X_8_Pin, coder & (0x00000001 << 7));

        HAL_GPIO_WritePin(X_9_GPIO_Port, X_9_Pin, (coder & (0x00000001 << 8)) && 1);
        HAL_GPIO_WritePin(X_10_GPIO_Port, X_10_Pin, (coder & (0x00000001 << 9)) && 1);
        HAL_GPIO_WritePin(X_11_GPIO_Port, X_11_Pin, (coder & (0x00000001 << 10)) && 1);
        HAL_GPIO_WritePin(X_12_GPIO_Port, X_12_Pin, (coder & (0x00000001 << 11)) && 1);
        HAL_GPIO_WritePin(X_13_GPIO_Port, X_13_Pin, (coder & (0x00000001 << 12)) && 1);
        HAL_GPIO_WritePin(X_14_GPIO_Port, X_14_Pin, (coder & (0x00000001 << 13)) && 1);
        HAL_GPIO_WritePin(X_15_GPIO_Port, X_15_Pin, (coder & (0x00000001 << 14)) && 1);
        HAL_GPIO_WritePin(X_16_GPIO_Port, X_16_Pin, (coder & (0x00000001 << 15)) && 1);
        HAL_GPIO_WritePin(X_17_GPIO_Port, X_17_Pin, (coder & (0x00000001 << 16)) && 1);
        HAL_GPIO_WritePin(X_18_GPIO_Port, X_18_Pin, (coder & (0x00000001 << 17)) && 1);
        HAL_GPIO_WritePin(X_19_GPIO_Port, X_19_Pin, (coder & (0x00000001 << 18)) && 1);
        HAL_GPIO_WritePin(X_20_GPIO_Port, X_20_Pin, (coder & (0x00000001 << 19)) && 1);
        HAL_GPIO_WritePin(X_21_GPIO_Port, X_21_Pin, (coder & (0x00000001 << 20)) && 1);
        HAL_GPIO_WritePin(X_22_GPIO_Port, X_22_Pin, (coder & (0x00000001 << 21)) && 1);
        // coder <<=1;
        // if(coder==0)    coder = 1;
        // HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
    while (1) {
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
