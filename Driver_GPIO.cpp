#include "Driver_GPIO.h"

void GPIO_PIN_Init(GPIO_TypeDef *nPort, int nPin, int Config, int State)
{
	  nPort->MODER  &= ~(GPIO_MODE_MASK << (2*nPin));
    nPort->MODER  |= (((Config >> GPIO_MODE_pos) & GPIO_MODE_MASK) << (2*nPin));

		nPort->OTYPER  &= ~(GPIO_OTYPE_MASK << (nPin));
    nPort->OTYPER  |= (((Config >> GPIO_OTYPE_pos) & GPIO_OTYPE_MASK) << (nPin));
	
		nPort->OSPEEDR  &= ~(GPIO_OSPEED_MASK << (2*nPin));
    nPort->OSPEEDR  |= (((Config >> GPIO_OSPEED_pos) & GPIO_OSPEED_MASK) << (2*nPin));
    
		nPort->PUPDR  &= ~(GPIO_PUPD_MASK << (2*nPin));
    nPort->PUPDR  |= (((Config >> GPIO_PUPD_pos) & GPIO_PUPD_MASK) << (2*nPin));
	
		if(nPin < 8)
		{
			nPort->AFR[0]  &= ~(GPIO_AFX_MASK << (4*nPin));
			nPort->AFR[0]  |= (((Config >> GPIO_AFX_pos) & GPIO_AFX_MASK) << (4*nPin));
		}
		else
		{
			nPin = nPin % 8;
			nPort->AFR[1]  &= ~(GPIO_AFX_MASK << (4*nPin));
			nPort->AFR[1]  |= (((Config >> GPIO_AFX_pos) & GPIO_AFX_MASK) << (4*nPin));
		}
}

void GPIO_PIN_SetState(GPIO_TypeDef *nPort, int nPin, int State)
{
	if(State == 0)
		nPort->ODR &= ~(1 << nPin);
	else if(State == 1)
		nPort->ODR |= (1 << nPin);
	else if(State == 2)
		nPort->ODR ^= (1 << nPin);
}

int GPIO_PIN_ReadState(GPIO_TypeDef *nPort, int nPin)
{
	if((nPort->IDR & (1 << nPin)) == 0)
		return 0;
	else
		return 1;
}
