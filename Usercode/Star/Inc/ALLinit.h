#ifndef __ALLINIT_H__
#define __ALLINIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void User_Init(void);
extern volatile uint32_t globe_time_ms;
extern volatile uint8_t motor_update_flag;
#ifdef __cplusplus
}
#endif

#endif /* __ALLINIT_H__ */
