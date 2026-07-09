/**
  ******************************************************************************
  * @file    star.c
  * @brief   CAN 接收中断回调 & 定时器中断回调
  *
  *          CAN FIFO0/FIFO1 消息接收：
  *            - 读取 CAN 消息，存入 rx_buffer / can_id
  *            - 置位 READ_FLAG 通知主循环
  *
  *          TIM1/TIM2 周期中断：
  *            - TIM1: 主控制循环 (1kHz)
  *            - TIM2: 辅助控制循环 (1kHz)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "star.h"
#include "ALLinit.h"
#include "DrMotor.h"

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  CAN FIFO0 消息接收回调
  * @note   当 FIFO0 有新消息时由 HAL_CAN_IRQHandler 调用
  * @param  hcan: CAN 句柄指针
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t *)rx_buffer) == HAL_OK)
    {
        can_id = RxHeader.StdId;

        if (state_pending)
        {
            /* 状态查询回复 → 直接解码 */
            motor_state_update_from_isr(can_id, (const uint8_t *)rx_buffer);
            state_pending = 0;
        }
        else
        {
            /* ACK 等其他消息 → 跳过, 不污染 motor_state */
            READ_FLAG = 1;
        }
    }
}

/**
  * @brief  CAN FIFO1 消息接收回调
  * @note   当 FIFO1 有新消息时由 HAL_CAN_IRQHandler 调用
  * @param  hcan: CAN 句柄指针
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, (uint8_t *)rx_buffer) == HAL_OK)
    {
        can_id    = RxHeader.StdId;
        READ_FLAG = 1;
    }
}

/**
  * @brief  定时器周期中断回调
  * @note   TIM1 和 TIM2 共用此回调，通过 htim->Instance 区分
  * @param  htim: 定时器句柄指针
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /* TIM1: 主控制逻辑 (1kHz) */
        globe_time_ms++;         /* 毫秒计数 (49.7 天不溢出) */

    }
    else if (htim->Instance == TIM2)
    {
        /* TIM2: 通知主循环更新电机控制 (1kHz) */
        motor_update_flag = 1;
    }
}
