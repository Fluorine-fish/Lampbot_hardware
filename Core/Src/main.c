/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote.h"
#include "Communications.h"
#include "Arm.h"
#include "PID.h"
#include "Arm_Calc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern RC_t RC;
extern CAN_TxHeaderTypeDef motor_tx_message;
extern uint8_t motor_can_send_data[8];
extern Arm_Params_t Arm_params;

extern DM_motor_t J4310_1;
extern M2006_motor_t M2006_1;
extern PID_Param PID_Speed_M2006_1;
extern PID_Param PID_Angle_M2006_1;

extern int32_t angle;
extern int32_t last_angle;

extern double Pos[4];
uint8_t Enable_flag = 0;
extern double Arm_params_input[4];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  //TIM5负责处理M2006的位置环PID
  if (htim == &htim5) {
    M2006_Angel(Pos[3]);
  }
  //TIM2负责处理机械臂计算 20Hz 抢占优先级低
  if(htim == &htim2) {
    Arm_Calculate(Arm_params_input[0],Arm_params_input[1],Arm_params_input[2],&Arm_params);
  }
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
extern uint8_t RC_Data[18];
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
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  HAL_UART_Receive_DMA(&huart3,RC_Data,sizeof(RC_Data));

  Arm_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // ReSharper disable once CppDFAEndlessLoop
  while (1)
  {
    if(RC.s1 == 3 && RC.s2 == 1) {
      //判断是否使能如果没有使能就使能
      if(!Enable_flag) {
        Arm_Motor_Enable();
      }

      if(RC.ch0 >= 100 && RC.ch0 <=660 ) {
        if(Pos[0] < 2) Pos[0] += 0.02;
        else Pos[0] = 2;
      }else if(RC.ch0 >= -660 && RC.ch0 <=-110) {
        if(Pos[0] > 0.5) Pos[0] -= 0.02;
        else Pos[0] = 0.5;
      }
      if(RC.ch1 >= 100 && RC.ch1 <=660 ) {
        if(Pos[1] < 2) Pos[1] += 0.02;
        else Pos[1] = 2;
      }else if(RC.ch1 >= -660 && RC.ch1 <=-110) {
        if(Pos[1] > 0.5) Pos[1] -= 0.02;
        else Pos[1] = 0.5;
      }
      if(RC.ch2 >= 100 && RC.ch2 <=660 ) {
        if(Pos[2] < 2) Pos[2] += 0.02;
        else Pos[2] = 2;
      }else if(RC.ch2 >= -660 && RC.ch2 <=-110) {
        if(Pos[2] > 0.6) Pos[2] -= 0.02;
        else Pos[2] = 0.6;
      }
      if(RC.ch3 >= 100 && RC.ch3 <=660 ) {
        if(Pos[3] < 2) Pos[3] += 0.02;
        else Pos[3] = 2;
      }else if(RC.ch3 >= -660 && RC.ch3 <=-110) {
        if(Pos[3] > 0.5) Pos[3] -= 0.02;
        else Pos[3] = 0.5;
      }

      DM_SpeedPosition_cmd(&hcan1,0x101,3.0,Pos[0]);
      HAL_Delay(5);
      DM_SpeedPosition_cmd(&hcan1,0x102,3.0,Pos[1]);
      HAL_Delay(5);
      DM_SpeedPosition_cmd(&hcan1,0x103,3.0,Pos[2]);
      HAL_Delay(5);
    }
    else if(RC.s1 == 3 && RC.s2 == 3) {
        //判断是否使能如果没有使能就使能
        if(!Enable_flag) {
          Arm_Motor_Enable();
      }
      for(uint8_t i = 0; i < 4; i++) {
        Pos[i] = 0.5;
      }
      Pos[2]=0.7;
      Pos[1]=1.0;
      DM_SpeedPosition_cmd(&hcan1,0x101,0.8,0.5);
      HAL_Delay(5);
      DM_SpeedPosition_cmd(&hcan1,0x102,0.8,1.0);
      HAL_Delay(5);
      DM_SpeedPosition_cmd(&hcan1,0x103,0.8,0.7);
      HAL_Delay(5);

    }
    else if(RC.s1 == 1 && RC.s2 == 1)
    {
      DM_SpeedPosition_cmd(&hcan1,0x101,0.8,0);
      HAL_Delay(5);
      DM_SpeedPosition_cmd(&hcan1,0x102,0.8,0);
      HAL_Delay(5);
      DM_SpeedPosition_cmd(&hcan1,0x103,0.8,0);
      HAL_Delay(5);
    }
    else {
      //判断是否失能如果没有失能就失能
      if(Enable_flag) {
        Arm_Motor_Disable();
      }
    }

    // if(RC.s1 == 3 && RC.s2 == 3)
    // {
    //   Pos[3] = 0.959931/1320*(RC.ch1 + 660);
    // }else
    // {
    //   Pos[3] = 0.0;
    // }

    HAL_Delay(50);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
