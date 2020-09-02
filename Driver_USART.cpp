#include "Driver_USART.h"

#define UART_RX_SIZE	128
#define UART_TX_SIZE	128
#define UART_RX_MASK	(UART_RX_SIZE - 1)
#define UART_TX_MASK	(UART_TX_SIZE - 1)

UART_Control_Type Uart1_Control;
UART_Control_Type Uart2_Control;
UART_Control_Type Uart3_Control;

uint8_t Uart1_RxBuffer[UART_RX_SIZE];
uint8_t Uart2_RxBuffer[UART_RX_SIZE];
uint8_t Uart3_RxBuffer[UART_RX_SIZE];

uint8_t Uart1_TxBuffer[UART_TX_SIZE];
uint8_t Uart2_TxBuffer[UART_TX_SIZE];
uint8_t Uart3_TxBuffer[UART_TX_SIZE];

void USART2_IRQHandler_Auxiliar(void);
void USART3_IRQHandler_Auxiliar(void);

void UART2_SendString(uint8_t *ptrBuffer)
{
	uint8_t Length = 0;
	uint8_t *pBuffer = ptrBuffer;
	
	while(Length < 255)
	{ 
		if(*pBuffer == 0x00)
			break;
		Length++;
		pBuffer++;
	}
	
	UART2_SendData((uint8_t *)ptrBuffer, Length);
}

void UART3_SendString(uint8_t *ptrBuffer)
{
	uint8_t Length = 0;
	uint8_t *pBuffer = ptrBuffer;
	
	while(Length < 255)
	{ 
		if(*pBuffer == 0x00)
			break;
		Length++;
		pBuffer++;
	}
	
	UART3_SendData((uint8_t *)ptrBuffer, Length);
}

//--------------------------------------//
void UART2_Print(uint8_t *ptrBuffer)
{
	UART2_SendString(ptrBuffer);
	
	while(Uart2_Control.TxLevel > 0);
	
//	while(( USART2->ISR & USART_ISR_TC ) == 0);
}

void UART3_Print(uint8_t *ptrBuffer)
{
	UART3_SendString(ptrBuffer);
	
	while(Uart3_Control.TxLevel > 0);
//	while(( USART3->ISR & USART_ISR_TC ) == 0);
}

//--------------------------------------//
void UART2_SendData(uint8_t *ptrBuffer, uint8_t Length)
{
	Uart2_Control.ptrTxBuffer = ptrBuffer;
	Uart2_Control.TxLevel = Length;
	
	if(Uart2_Control.TxLevel > 0)
	{
		USART2->TDR = *Uart2_Control.ptrTxBuffer++;
		Uart2_Control.TxLevel--;
		// TXIDLE int enable
		USART2->CR1 |= USART_CR1_TXEIE;
	}
}

void UART3_SendData(uint8_t *ptrBuffer, uint8_t Length)
{
	Uart3_Control.ptrTxBuffer = ptrBuffer;
	Uart3_Control.TxLevel = Length;
	
	if(Uart3_Control.TxLevel > 0)
	{
		USART3->TDR = *Uart3_Control.ptrTxBuffer++;
		Uart3_Control.TxLevel--;
		// TXIDLE int enable
		USART3->CR1 |= USART_CR1_TXEIE;
	}
}

//--------------------------------------//
uint8_t UART2_ReadByte(void)
{
	uint8_t data = 0;
	
	USART2->CR1 &= ~USART_CR1_RXNEIE;	// Deshabilita interrupcion de recepcion
	
	if(Uart2_Control.RxLevel > 0)
	{
		Uart2_Control.cRxOut &= UART_RX_MASK;
		data = Uart2_Control.ptrRxBuffer[Uart2_Control.cRxOut++];
		Uart2_Control.RxLevel--;
	}
	
	USART2->CR1 |= USART_CR1_RXNEIE;	// Habilita interrupcion de recepcion
	
	return data;
}

uint8_t UART3_ReadByte(void)
{
	uint8_t data = 0;
	
	USART3->CR1 &= ~USART_CR1_RXNEIE;	// Deshabilita interrupcion de recepcion
	
	if(Uart3_Control.RxLevel > 0)
	{
		Uart3_Control.cRxOut &= UART_RX_MASK;
		data = Uart3_Control.ptrRxBuffer[Uart3_Control.cRxOut++];
		Uart3_Control.RxLevel--;
	}
	
	USART3->CR1 |= USART_CR1_RXNEIE;	// Habilita interrupcion de recepcion
	
	return data;
}

void UART2_Flush(void)
{
	USART2->CR1 &= ~USART_CR1_RXNEIE;	// Deshabilita interrupcion de recepcion
	
	while(Uart2_Control.RxLevel > 0)
	{
		Uart2_Control.cRxOut &= UART_RX_MASK;
		Uart2_Control.cRxOut++;
		Uart2_Control.RxLevel--;
	}
	
	USART2->CR1 |= USART_CR1_RXNEIE;	// Habilita interrupcion de recepcion
}

void UART3_Flush(void)
{
	USART3->CR1 &= ~USART_CR1_RXNEIE;	// Deshabilita interrupcion de recepcion
	
	while(Uart3_Control.RxLevel > 0)
	{
		Uart3_Control.cRxOut &= UART_RX_MASK;
		Uart3_Control.cRxOut++;
		Uart3_Control.RxLevel--;
	}
	
	USART3->CR1 |= USART_CR1_RXNEIE;	// Habilita interrupcion de recepcion
}
//--------------------------------------//
int UART2_IsTXReady(void)
{
	if(Uart2_Control.TxLevel > 0)
		return 0;
	if(( USART2->ISR & USART_ISR_TC ) == 0)
		return 0;
	
	return 1;
}

int UART3_IsTXReady(void)
{
	if(Uart3_Control.TxLevel > 0)
		return 0;
//	if(( USART3->ISR & USART_ISR_TC ) == 0)
	//	return 0;
	
	return 1;
}
//--------------------------------------//
void UART2_WriteChar(uint8_t chrValue)
{
	while(Uart2_Control.TxLevel > 0);
	while(( USART2->ISR & USART_ISR_TC ) == 0);
	
	USART2->TDR = chrValue;
}

void UART3_WriteChar(uint8_t chrValue)
{
	while(Uart3_Control.TxLevel > 0);
	while(( USART3->ISR & USART_ISR_TC ) == 0);
	
	USART3->TDR = chrValue;
}
//--------------------------------------//
uint8_t UART2_IsAvailable(void)
{
	return	Uart2_Control.RxLevel;
}

uint8_t UART3_IsAvailable(void)
{
	return	Uart3_Control.RxLevel;
}
//--------------------------------------//
uint8_t UART2_TxLevel(void)
{
	return	Uart2_Control.TxLevel;
}

uint8_t UART3_TxLevel(void)
{
	return	Uart3_Control.TxLevel;
}
//--------------------------------------//
void UART_EnableBridge(int Origen, int Destination)
{
	UART_Control_Type *pUartControl;
	
	if(Origen == 1)
		pUartControl = &Uart1_Control;
	else if(Origen == 2)
		pUartControl = &Uart2_Control;
	else if(Origen == 3)
		pUartControl = &Uart3_Control;
	else return;
	
	if(Destination == 1)
		pUartControl->BridgeTo1 = 1;
	else if(Destination == 2)
		pUartControl->BridgeTo2 = 1;
	else if(Destination == 3)
		pUartControl->BridgeTo3 = 1;
	else return;
}
//--------------------------------------//
void UART_DisableBridge(int Origen, int Destination)
{
	UART_Control_Type *pUartControl;
	
	if(Origen == 1)
		pUartControl = &Uart1_Control;
	else if(Origen == 2)
		pUartControl = &Uart2_Control;
	else if(Origen == 3)
		pUartControl = &Uart3_Control;
	else return;
	
	if(Destination == 1)
		pUartControl->BridgeTo1 = 0;
	else if(Destination == 2)
		pUartControl->BridgeTo2 = 0;
	else if(Destination == 3)
		pUartControl->BridgeTo3 = 0;
	else return;
}
//--------------------------------------//
void ResetUartControl(UART_Control_Type *pUartControl, uint8_t *pRxBuffer, uint8_t TxSize, uint8_t RxSize)
{
		pUartControl->ptrRxBuffer = pRxBuffer;
		pUartControl->ptrTxBuffer = 0;
		pUartControl->RxLevel = 0;
		pUartControl->TxLevel = 0;
		pUartControl->RxMaxSize = RxSize;
		pUartControl->TxMaxSize = TxSize;
		pUartControl->cRxIn = 0;
		pUartControl->cRxOut = 0;
		pUartControl->BridgeTo1 = 0;
		pUartControl->BridgeTo2 = 0;
		pUartControl->BridgeTo3 = 0;
}
//--------------------------------------//
USART_TypeDef *GetUARTType(uint32_t UartId)
{
	if(UartId == 1)
		return USART1;
	if(UartId == 2)
		return USART2;
	if(UartId == 3)
		return USART3;
	
	return 0;
}
//--------------------------------------//
void UART_Init(UART_Config_Type *uartConfig)
{
USART_TypeDef *pUart;
	uint32_t err, uart_fra_multiplier, baudRateGenerator;
	uint32_t _PeripheralClock;
	
	pUart =  GetUARTType(uartConfig->UartId);
	if(pUart == 0)
		return;
	
	//SystemCoreClock
	//36 MHz max for APB1
	//72 MHz max for APB2
	
	// Revisar RCC->CFGR

    //SystemCoreClock = 6000000;

	if(pUart == USART1)
		_PeripheralClock = SystemCoreClock / 4;
	else if(pUart == USART2)
		_PeripheralClock = SystemCoreClock / 4;
	else if(pUart == USART3)
		_PeripheralClock = SystemCoreClock / 4;

	//if(uartConfig->Baudrate > 250000)
	//	uartConfig->Baudrate = 250000;
	
	if(uartConfig->Baudrate < 480)
		uartConfig->Baudrate = 480;
	
	if(uartConfig->NBits > 9)
		uartConfig->NBits = 9;
	
	if(uartConfig->Stopbits > 2)
		uartConfig->Stopbits = 2;
	
	// Enable Clk for USART & Reset USART
	if(uartConfig->UartId == 1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
	}
	else if(uartConfig->UartId == 2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
	}
	else if(uartConfig->UartId == 3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
	}
	else
		return;
	
	// USART Baud Generator
	baudRateGenerator = (_PeripheralClock / 16) / (uartConfig->Baudrate);
	err = _PeripheralClock - baudRateGenerator * 16 * (uartConfig->Baudrate);
	uart_fra_multiplier = (err * 0xF) / (baudRateGenerator * 16 * (uartConfig->Baudrate));
	pUart->BRR = (baudRateGenerator << 4) | (uart_fra_multiplier & 0xF);	/* baud rate */
		
	// USART Config
	//pUart->CFG = (0 << 0) | ((uartConfig->NBits - 7) << 2) | (uartConfig->Parity << 4) | (uartConfig->Stopbits << 6);
	
	//pUart->CR1 = (USART_CR1_SBK & 0)|(USART_CR1_RWU & 0)|(USART_CR1_RE & 0)|(USART_CR1_TE & 0)|
    pUart->CR1 = (0 & 0)|(0 & 0)|(USART_CR1_RE & 0)|(USART_CR1_TE & 0)|
								(USART_CR1_IDLEIE & 0)|(USART_CR1_RXNEIE & 0)|(USART_CR1_TCIE & 0)|(USART_CR1_TXEIE & 0)|
								(USART_CR1_PEIE & 0)|(USART_CR1_PS & 0)|(USART_CR1_PCE & 0)|(USART_CR1_WAKE & 0)|
								(USART_CR1_M & 0)|(USART_CR1_UE & 0)|(14 & 0)|(USART_CR1_OVER8 & 0);
								
	pUart->CR2 = (USART_CR2_ADD & 0)|(USART_CR2_LBDL & 0)|(USART_CR2_LBDIE & 0)|
								(USART_CR2_LBCL & 0)|(USART_CR2_CPHA & 0)|(USART_CR2_CPOL & 0)|(USART_CR2_CLKEN & 0)|
								(USART_CR2_STOP & 0)|(USART_CR2_LINEN & 0);
	pUart->CR3 = 0;
	pUart->GTPR = 0;
	
	// USART enable
	pUart->CR1 |= (USART_CR1_UE & ~0);
	
	// USART RX & TX enable
	pUart->CR1 |= (USART_CR1_RE & ~0);
	pUart->CR1 |= (USART_CR1_TE & ~0);
	
	// RX int enabled
	pUart->CR1 |= USART_CR1_RXNEIE;	// Habilita interrupcion de recepcion
	
	// TX int disabled
	pUart->CR1 &= ~USART_CR1_TXEIE;	// Deshabilita interrupcion de transmision (TX Empty)
	pUart->CR1 &= ~USART_CR1_TCIE;	// Deshabilita interrupcion de recepcion (TX Completed)
	
	// Configuracion del controlador de interrupciones (NVIC)
	if(uartConfig->UartId == 1)
	{
		ResetUartControl(&Uart1_Control,
										&Uart1_RxBuffer[0],
										UART_TX_SIZE,
										UART_RX_SIZE);
		NVIC_SetPriority(USART1_IRQn, 2);	
		NVIC_EnableIRQ(USART1_IRQn);							// Habilitacion de la interrupcion
	}
	
	if(uartConfig->UartId == 2)
	{
		ResetUartControl(&Uart2_Control,
										&Uart2_RxBuffer[0],
										UART_TX_SIZE,
										UART_RX_SIZE);
		NVIC_SetPriority(USART2_IRQn, 2);	
        NVIC_SetVector(USART2_IRQn, (uint32_t)USART2_IRQHandler_Auxiliar);
		NVIC_EnableIRQ(USART2_IRQn);							// Habilitacion de la interrupcion
	}
	
	if(uartConfig->UartId == 3)
	{
		ResetUartControl(&Uart3_Control,
										&Uart3_RxBuffer[0],
										UART_TX_SIZE,
										UART_RX_SIZE);
		NVIC_SetPriority(USART3_IRQn, 2);
        NVIC_SetVector(USART3_IRQn, (uint32_t)USART3_IRQHandler_Auxiliar);	
		NVIC_EnableIRQ(USART3_IRQn);							// Habilitacion de la interrupcion
	}
}
//--------------------------------------//
void UART_GetDefaultSetting(UART_Config_Type *pUartConfig)
{
	pUartConfig->Baudrate = 9600;
	pUartConfig->NBits = 8;
	pUartConfig->Parity = 0;
	pUartConfig->Stopbits = 1;
	pUartConfig->UartId = 0;
}
//--------------------------------------//
//--------------------------------------//
void USART2_IRQHandler_Auxiliar(void)
{
	USART_TypeDef *pUsart;
	UART_Control_Type *pUsartControl;
	uint32_t usartSTS;
	uint8_t Dummy;
	
	pUsart = USART2;
	pUsartControl = &Uart2_Control;
	
	usartSTS = pUsart->ISR;
	
	// RXRDY interruption
	if(usartSTS & USART_ISR_RXNE)
	{
		Dummy = pUsart->RDR;
	
		pUsartControl->cRxIn &= UART_RX_MASK;
		pUsartControl->ptrRxBuffer[pUsartControl->cRxIn++] = Dummy;
		if(pUsartControl->RxLevel < UART_RX_SIZE)
			pUsartControl->RxLevel++;
		else
		{
			pUsartControl->cRxOut++;
			pUsartControl->cRxOut &= UART_RX_MASK;
		}
		//TimeOut_Rx = TIMEOUT_RX ;
	}
	
	// TXIDLE interruption
	if(usartSTS & USART_ISR_TXE)
	{
		if(pUsartControl->TxLevel > 0)
		{
			pUsart->TDR = *pUsartControl->ptrTxBuffer++;
			pUsartControl->TxLevel--;
		}
		else
		{
			// TXIDLE int disable
			pUsart->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

void USART3_IRQHandler_Auxiliar(void)
{
	USART_TypeDef *pUsart;
	UART_Control_Type *pUsartControl;
	uint32_t usartSTS;
	uint8_t Dummy;
	
	pUsart = USART3;
	pUsartControl = &Uart3_Control;
	
	usartSTS = pUsart->ISR;
	
	// RXRDY interruption
	if(usartSTS & USART_ISR_RXNE)
	{
		Dummy = pUsart->RDR;
	
		pUsartControl->cRxIn &= UART_RX_MASK;
		pUsartControl->ptrRxBuffer[pUsartControl->cRxIn++] = Dummy;
		if(pUsartControl->RxLevel < UART_RX_SIZE)
			pUsartControl->RxLevel++;
		else
		{
			pUsartControl->cRxOut++;
			pUsartControl->cRxOut &= UART_RX_MASK;
		}
		//TimeOut_Rx = TIMEOUT_RX ;
	}
	
	// TXIDLE interruption
	if(usartSTS & USART_ISR_TXE)
	{
		if(pUsartControl->TxLevel > 0)
		{
			pUsart->TDR = *pUsartControl->ptrTxBuffer++;
			pUsartControl->TxLevel--;
		}
		else
		{
			// TXIDLE int disable
			pUsart->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}
