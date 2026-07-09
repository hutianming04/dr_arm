/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bspcan.h"
#include "ALLinit.h"
#include "DrMotor.h"
#include "pid.h"
#include "impedance.h"
#include "differentiator.h"
#include "VOFA.h"
#include <math.h>
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

/* USER CODE BEGIN PV */
float target = 0.0f;              /* 全局目标角度 */
//PID_TypeDef motor_pid;            /* 电机位置环 PID */
Impedance_TypeDef motor_impedance;/* 阻抗控制器 */
Differentiator_TypeDef target_diff; /* 目标位置微分器 → 期望速度 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  User_Init();

  /* 初始化位置环 PID: Kp=Ki=Kd=0, Ts=1ms, tau=0(不滤波), 输出±16384 */
  //PID_Init(&motor_pid, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, 16384.0f, -16384.0f);

  /* 初始化阻抗控制器: 
   *   控制律: torque = Kp*(pos - pos_) + t_ff + Kd*(vel - vel_)
   *     pos  = 虚拟期望位置,  pos_ = 实际位置反馈
   *     vel  = 虚拟期望速度,  vel_ = 实际速度反馈      */
  Impedance_Init(&motor_impedance, 0.0f, 0.00f, 36.0f, -36.0f);

  /* 初始化微分器: Ts=1ms, tau=0.25ms → alpha=Ts/(Ts+tau)=0.8 */
  Diff_Init(&target_diff, 0.001f, 0.00025f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint8_t query_toggle = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (motor_update_flag)
    {
        motor_update_flag = 0;

        /* 更新目标轨迹 */
        float t = globe_time_ms * 0.001f;          /* ms → s */
        //target = 180.0f * sinf(1.4f * t);          /* 虚拟期望位置, ±180°, 周期≈4.5s */
        float target_vel = Diff_Update(&target_diff, target); /* 微分器求期望速度 (°/s) */

        /* ── 更新电机数据 & 多圈解算 (使用上周期 ISR 更新的 motor_state) ── */
        motor_update_infinite_angle(&motor.data, motor_state[0][0]);
        motor.data.vel = motor_state[0][1]; /* r/min */

        /* ── 阻抗控制器 → 力矩指令 ── */
        motor.data.aim   = target;
        motor.data.error = target - motor.data.angle_infinite;
        float torque_cmd = Impedance_Update(&motor_impedance,
            target,                        /* pos  — 虚拟期望位置 */
            motor.data.angle_infinite,     /* pos_ — 实际位置反馈 */
            target_vel,                    /* vel  — 虚拟期望速度 (°/s) */
            motor.data.vel,                /* vel_ — 实际速度反馈 (r/min) */
            0.0f);                         /* t_ff — 前馈力矩 */

        /* ── 1. 先查状态 @500Hz (query 先发, state 回复先到, ACK 后到) ── */
        query_toggle = !query_toggle;
        if (query_toggle)
            query_state_async(1);    /* 设 state_pending=1, 非阻塞发送 0x1E */

        /* ── 2. 再发扭矩指令 (ACK 后到, ISR 里 state_pending 已清零 → 跳过) ── */
        set_torque_cmd(1, torque_cmd);              /* id=1, 非阻塞力矩发送 */

        /* VOFA+ 上位机波形: 期望位置 vs 实际位置 */
        VOFA_justfloat(target, motor.data.angle_infinite,
                       motor_state[0][1], torque_cmd,
                       0, 0, 0, 0, 0, 0);
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
