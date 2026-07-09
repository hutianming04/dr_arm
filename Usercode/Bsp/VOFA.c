/*
 * @Descripttion: VOFA+ JustFloat 协议串口发送
 * @version: v4 — USART 中断模式，简单可靠
 * @Author: Eugene / Andy
 * @Date: 2024-04-09 13:26:42
 * @LastEditTime: 2026-07-09
 */
#include "VOFA.h"
#include "usart.h"

union vofa_Transmit
{
    float data1[10];
    uint8_t data2[44];      /* 10 float + 4 尾帧 = 44 字节 */
} data;

static uint8_t tx_busy = 0;  /* 中断发送忙标志 */

/**
 * @brief VOFA+ 使用串口中断发送 JustFloat 格式数据
 *
 *   44 字节 @921600 ≈ 0.48ms，1kHz 无压力。
 *   上次没发完自动跳过，不阻塞控制循环。
 */
void VOFA_justfloat(float a, float b, float c, float d,
                    float e, float f, float g, float h,
                    float j, float k)
{
    if (tx_busy)
        return;  /* 上次没发完，跳过 */

    uint8_t i = 0;
    data.data1[i++] = a;
    data.data1[i++] = b;
    data.data1[i++] = c;
    data.data1[i++] = d;
    data.data1[i++] = e;
    data.data1[i++] = f;
    data.data1[i++] = g;
    data.data1[i++] = h;
    data.data1[i++] = j;
    data.data1[i++] = k;
    /* 尾帧 */
    data.data2[40] = 0x00;
    data.data2[41] = 0x00;
    data.data2[42] = 0x80;
    data.data2[43] = 0x7f;

    tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart1, data.data2, sizeof(data.data2)) != HAL_OK)
        tx_busy = 0;  /* 启动失败, 还原标志 */
}

/**
 * @brief 发送完成回调（由 HAL ISR 调用）
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        tx_busy = 0;
}
