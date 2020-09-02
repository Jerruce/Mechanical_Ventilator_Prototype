#ifndef _DRIVER_USART_H_
#define _DRIVER_USART_H_

#include "mcu.h"

#define UART_PARITY_NONE 0
#define UART_PARITY_EVEN 2
#define UART_PARITY_ODD  3

//#define UART_BITS_7 0
//#define UART_BITS_8 1
//#define UART_BITS_9 2

//#define UART_STOP_1 0
//#define UART_STOP_2 1

typedef struct _UART_Config
{
	uint8_t UartId;   // Port Number
	uint8_t NBits;   			// Port Number
	uint8_t Parity;
	uint8_t Stopbits;
  uint32_t Baudrate;    // Port Number
} UART_Config_Type;

typedef struct _UART_Control
{
	uint8_t		*ptrRxBuffer;
	uint8_t		*ptrTxBuffer;
	uint8_t		TxLevel;
	uint8_t		RxLevel;
	uint8_t		TxMaxSize;
	uint8_t		RxMaxSize;
	uint8_t 	cRxIn;
	uint8_t 	cRxOut;
	uint8_t 	BridgeTo1;
	uint8_t 	BridgeTo2;
	uint8_t 	BridgeTo3;
} UART_Control_Type;

void UART3_SendString(uint8_t *ptrBuffer);
void UART3_Print(uint8_t *ptrBuffer);
void UART3_WriteChar(uint8_t chrValue);
void UART3_SendData(uint8_t *ptrBuffer, uint8_t Length);
uint8_t UART3_ReadByte(void);
void UART3_Flush(void);
int UART3_IsTXReady(void);
uint8_t UART3_IsAvailable(void);
uint8_t UART3_TxLevel(void);


void UART2_SendString(uint8_t *ptrBuffer);
void UART2_Print(uint8_t *ptrBuffer);
void UART2_WriteChar(uint8_t chrValue);
void UART2_SendData(uint8_t *ptrBuffer, uint8_t Length);
uint8_t UART2_ReadByte(void);
void UART2_Flush(void);
int UART2_IsTXReady(void);
uint8_t UART2_IsAvailable(void);
uint8_t UART2_TxLevel(void);



void UART_EnableBridge(int Origen, int Destination);
void UART_DiableBridge(int Origen, int Destination);

void UART_Init(UART_Config_Type *uartConfig);

void UART_GetDefaultSetting(UART_Config_Type *pUartConfig);

#endif /* _DRIVER_USART_H_ */
