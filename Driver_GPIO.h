#ifndef _DRIVER_GPIO_H_
#define _DRIVER_GPIO_H_

#include "mcu.h"

#define GPIO_MODE_pos 	(0)
#define GPIO_MODE_MASK 	(0x3)
#define GPIO_MODE_IN 		(0 << GPIO_MODE_pos)
#define GPIO_MODE_OUT 	(1 << GPIO_MODE_pos)
#define GPIO_MODE_AFX 	(2 << GPIO_MODE_pos)
#define GPIO_MODE_AN 		(3 << GPIO_MODE_pos)

#define GPIO_OTYPE_pos 	(2)
#define GPIO_OTYPE_MASK (0x1)
#define GPIO_OTYPE_PP 	(0 << GPIO_OTYPE_pos)
#define GPIO_OTYPE_OD 	(1 << GPIO_OTYPE_pos)

#define GPIO_OSPEED_pos 		(3)
#define GPIO_OSPEED_MASK 		(0x3)
#define GPIO_OSPEED_LOW 		(0 << GPIO_OSPEED_pos)
#define GPIO_OSPEED_MEDIUM 	(1 << GPIO_OSPEED_pos)
#define GPIO_OSPEED_HIGH 		(2 << GPIO_OSPEED_pos)
#define GPIO_OSPEED_VHIGH		(3 << GPIO_OSPEED_pos)

#define GPIO_PUPD_pos 			(5)
#define GPIO_PUPD_MASK 		(0x3)
#define GPIO_PUPD_NONE 		(0 << GPIO_PUPD_pos)
#define GPIO_PUPD_PU 			(1 << GPIO_PUPD_pos)
#define GPIO_PUPD_PD 			(2 << GPIO_PUPD_pos)
#define GPIO_PUPD_RESERVED	(3 << GPIO_PUPD_pos)

#define GPIO_AFX_pos 				(7)
#define GPIO_AFX_MASK 			(0xF)
#define GPIO_AFX_0		 			(0 << GPIO_AFX_pos)
#define GPIO_AFX_1 					(1 << GPIO_AFX_pos)
#define GPIO_AFX_2 					(2 << GPIO_AFX_pos)
#define GPIO_AFX_3					(3 << GPIO_AFX_pos)
#define GPIO_AFX_4		 			(4 << GPIO_AFX_pos)
#define GPIO_AFX_5 					(5 << GPIO_AFX_pos)
#define GPIO_AFX_6 					(6 << GPIO_AFX_pos)
#define GPIO_AFX_7					(7 << GPIO_AFX_pos)
#define GPIO_AFX_8		 			(8 << GPIO_AFX_pos)
#define GPIO_AFX_9 					(9 << GPIO_AFX_pos)
#define GPIO_AFX_10 				(10 << GPIO_AFX_pos)
#define GPIO_AFX_11					(11 << GPIO_AFX_pos)
#define GPIO_AFX_12					(12 << GPIO_AFX_pos)
#define GPIO_AFX_13					(13 << GPIO_AFX_pos)
#define GPIO_AFX_14					(14 << GPIO_AFX_pos)
#define GPIO_AFX_15					(15 << GPIO_AFX_pos)

void GPIO_PIN_Init(GPIO_TypeDef *nPort, int nPin, int Config, int State);
void GPIO_PIN_SetState(GPIO_TypeDef *nPort, int nPin, int State);
int GPIO_PIN_ReadState(GPIO_TypeDef *nPort, int nPin);

#endif /* _DRIVER_GPIO_H_ */
