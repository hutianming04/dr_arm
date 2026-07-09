/**
  ******************************************************************************
  * @file    ALLinit.c
  * @brief   用户初始化函数
  *
  *          User_Init() 汇总所有用户层初始化操作：
  *            - 启动定时器中断
  *            - 配置 CAN 滤波器
  *            - 启动 CAN 接收
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ALLinit.h"
#include "bspcan.h"
#include "can.h"
#include "tim.h"

/**
  * @brief  用户初始化函数
  * @note   在 main() 中完成外设初始化后调用
  *         汇总所有用户层初始化操作
  * @retval None
  */
 volatile uint32_t globe_time_ms = 0;
 volatile uint8_t motor_update_flag = 0;
void User_Init(void)
{
    /* 启动定时器中断 (TIM1 & TIM2, 均为 1kHz) */
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);

    /* 启动 CAN 外设 */
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 配置 CAN 滤波器 (接收所有标准帧, 存入 FIFO0) */
    BSP_CAN_Filter_Init();
}
