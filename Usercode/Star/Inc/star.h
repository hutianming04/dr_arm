#ifndef __STAR_H__
#define __STAR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"

/* Exported variables --------------------------------------------------------*/
extern volatile uint8_t  rx_buffer[8];
extern volatile uint16_t can_id;
extern volatile int8_t   READ_FLAG;

/* Exported functions --------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __STAR_H__ */
