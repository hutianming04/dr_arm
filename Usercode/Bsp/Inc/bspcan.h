#ifndef __BSPCAN_H__
#define __BSPCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"

/* Exported functions --------------------------------------------------------*/
void BSP_CAN_Filter_Init(void);
uint8_t Can_Send_Msg(uint32_t id, uint8_t len, uint8_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __BSPCAN_H__ */
