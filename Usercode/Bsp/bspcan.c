#include "bspcan.h"

/**
  * @brief  CAN滤波器初始化
  * @note   使用掩码模式，接收所有标准帧ID（不滤波）
  * @retval None
  */
void BSP_CAN_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                          /* 使用滤波器组 0 */
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;      /* 标识符掩码模式 */
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;     /* 32位位宽 */
    sFilterConfig.FilterIdHigh = 0x0000;                   /* ID 高 16 位 */
    sFilterConfig.FilterIdLow = 0x0000;                    /* ID 低 16 位 */
    sFilterConfig.FilterMaskIdHigh = 0x0000;               /* 掩码高 16 位 (0 = 不关心) */
    sFilterConfig.FilterMaskIdLow = 0x0000;                /* 掩码低 16 位 (0 = 不关心) */
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;     /* 匹配的消息放入 FIFO0 */
    sFilterConfig.FilterActivation = ENABLE;               /* 使能滤波器 */
    sFilterConfig.SlaveStartFilterBank = 14;               /* (CAN1 only, 无关) */

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  CAN 消息发送函数
  * @param  id     标准帧 ID (11 位)
  * @param  len    数据长度 (0~8 字节)
  * @param  pData  指向发送数据缓冲区的指针
  * @retval HAL_StatusTypeDef  HAL_OK 表示发送成功
  */
uint8_t Can_Send_Msg(uint32_t id, uint8_t len, uint8_t *pData)
{
    CAN_TxHeaderTypeDef   TxHeader;
    uint32_t              TxMailbox;

    if (pData == NULL || len > 8)
    {
        return HAL_ERROR;
    }

    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, pData, &TxMailbox) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
