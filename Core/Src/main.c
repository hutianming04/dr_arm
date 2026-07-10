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
#include "VOFA.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Ts = 1ms, alpha = 0.8 -> tau = Ts*(1-alpha)/alpha = 0.00025 */
#define TAU_ALPHA_08  0.00025f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define MOTOR_COUNT 4
float target[MOTOR_COUNT] = {0};              /* 全局目标角度 (°), 每个电机独立 */
PID_TypeDef pid_pos[MOTOR_COUNT];             /* 外环: 位置 → 速度指令 (r/min), 每个电机独立 */
PID_TypeDef pid_vel[MOTOR_COUNT];             /* 内环: 速度 → 力矩指令 (Nm), 每个电机独立 */
uint8_t motor_id_list[MOTOR_COUNT] = {2, 3, 4, 5};  /* 电机 ID 列表 */
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

  /* ── 5 电机双环 PID 初始化 ──
   *
   *  PID 参数默认 0 (Kp=Ki=Kd=0)，限幅和滤波与原有配置一致。
   *  外环 (位置): tau_sp=0(不滤波), tau_d=TAU_ALPHA_08(α=0.8), out=±3600 r/min
   *  内环 (速度): tau_sp=TAU_ALPHA_08(α=0.8), tau_d=0(不滤波), out=±3600 Nm
   */

  /* ── 电机 2 (idx=0) ── */
  PID_Init(&pid_pos[0], 0.0f, 0.0f, 0.0f, 0.001f,
           0.0f, TAU_ALPHA_08, 3600.0f, -3600.0f);
  PID_Init(&pid_vel[0], 0.0f, 0.0f, 0.0f, 0.001f,
           TAU_ALPHA_08, 0.0f, 3600.0f, -3600.0f);

  /* ── 电机 3 (idx=1) ── */
  PID_Init(&pid_pos[1], 0.0f, 0.0f, 0.0f, 0.001f,
           0.0f, TAU_ALPHA_08, 3600.0f, -3600.0f);
  PID_Init(&pid_vel[1], 0.0f, 0.0f, 0.0f, 0.001f,
           TAU_ALPHA_08, 0.0f, 3600.0f, -3600.0f);

  /* ── 电机 4 (idx=2) ── */
  PID_Init(&pid_pos[2], 0.0f, 0.0f, 0.0f, 0.001f,
           0.0f, TAU_ALPHA_08, 3600.0f, -3600.0f);
  PID_Init(&pid_vel[2], 0.0f, 0.0f, 0.0f, 0.001f,
           TAU_ALPHA_08, 0.0f, 3600.0f, -3600.0f);

  /* ── 电机 5 (idx=3) ── */
  PID_Init(&pid_pos[3], 0.0f, 0.0f, 0.0f, 0.001f,
           0.0f, TAU_ALPHA_08, 3600.0f, -3600.0f);
  PID_Init(&pid_vel[3], 0.0f, 0.0f, 0.0f, 0.001f,
           TAU_ALPHA_08, 0.0f, 3600.0f, -3600.0f);
  enable_angle_speed_torque_state(4);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 低速配置任务，不与 1kHz 快循环争抢 CAN 总线 */
    {
        static uint32_t last_cfg_ms = 0;
        if (globe_time_ms - last_cfg_ms >= 100)
        {
            last_cfg_ms = globe_time_ms;
            enable_angle_speed_torque_state(4);
        }
    }

    if (motor_update_flag)
    {
        motor_update_flag = 0;
        float t = (float)globe_time_ms * 0.001f; /* 秒 */

        /* TDM 轮询: 每 1ms 只控一个电机, 4ms 一轮 (电机 2/3/4/5) */
        uint8_t idx  = globe_time_ms % MOTOR_COUNT;
        uint8_t id   = motor_id_list[idx];
        uint8_t sidx = id - 1;  /* motor_state 索引 */

       // target[idx] = 15.0f * sinf(2.0f * 3.1415926f * 0.1f * t);

        /* ── 串级 PID ── */
        float vel_cmd    = PID_Update(&pid_pos[idx], target[idx], motor_state[sidx][0]);
        float torque_cmd = PID_Update(&pid_vel[idx], vel_cmd, motor_state[sidx][1]);

        set_torque(id, torque_cmd, 0.0f, 1);

        if (id == 4)
            HAL_Delay(1);  /* 等电机 4 回复帧收完，不干扰电机 5 */

        VOFA_justfloat(target[0], motor_state[1][0],
                       motor_state[1][1], torque_cmd,
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
