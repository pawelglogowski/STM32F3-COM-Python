
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MESSAGE_H
#define __MESSAGE_H

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/
typedef struct{
	portCHAR ucMessageID;
	uint8_t ucData[ 255 ];
	uint8_t ucLen;
} AMessage;

#endif /* __MESSAGE_H */
