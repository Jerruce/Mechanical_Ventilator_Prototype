

/* File inclusion */
#include "mbed.h"
#include "Driver_GPIO.h"
#include "stepper_position_driver.h"



/* Variable definition */

extern int posicion_actual_insp; // just for testing
extern int posicion_actual_exp; // just for testing

int m_SetPoint_1;
int m_SetPoint_2;

int m_Driver1_CMD;
int m_Driver2_CMD;

int m_Motor1_MinPos;
int m_Motor2_MinPos;


/* Function declaration */
void Timer11_Init(void);
void Timer14_Init(void);
void Timer1_Init(void);
void Timer8_Init(void);
void TIM1_TRG_COM_TIM11_IRQHandler_Auxiliar(void);
void TIM8_TRG_COM_TIM14_IRQHandler_Auxiliar(void);
void Mapeado(void);


/* Function definition */

void Timer11_Init(void)
{
	// Pulse pin: PE9
	// Enable pin: PF13
	// Direction pin: PF12
	TIM_TypeDef *pTIM;
	pTIM = TIM11;
	
	RCC->APB2ENR |= (1 << 18);
	RCC->APB2RSTR |= (1 << 18);
	RCC->APB2RSTR &= ~(1 << 18);
	
  pTIM->CR1 = 0;
	//pTIM->ARR = 1000 - 1; // 1000us
    pTIM->ARR = 62 - 1; // freq = 16 KHz
	//pTIM->BDTR = (15 << 1);	// Main output enable
	pTIM->CCER = (1 << 0);
	pTIM->CCMR1 = (0 << 0)|(1 << 3)|(0x7 << 4);
	pTIM->CCR1 = pTIM->ARR/2;
	pTIM->CNT = 0;
	pTIM->CR1 = (0 << 0)|(0 << 1)|(1 << 2)|(0 << 3)|(0 << 4)|(0 << 5)|(0 << 7)|(0 << 8)|(0 << 11);
	//pTIM->DCR = 0;
	pTIM->DIER = (1 << 0); // Update interrupt enabled
	//pTIM->DMAR = 0;
	pTIM->EGR = (1 << 0);		// Event generation
	pTIM->OR = 0;
	//pTIM->PSC = 216 - 1;
    pTIM->PSC = 6912 - 1;
	//pTIM->RCR = 0;
	//pTIM->SMCR = 0;
	pTIM->SR = 0xFFFF;
	
	NVIC_ClearPendingIRQ (TIM1_TRG_COM_TIM11_IRQn);
	
	pTIM->CR1 |= (1 << 0);
  //NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 4, 1);
  NVIC_SetVector(TIM1_TRG_COM_TIM11_IRQn, (uint32_t)TIM1_TRG_COM_TIM11_IRQHandler_Auxiliar);
	
  NVIC_EnableIRQ (TIM1_TRG_COM_TIM11_IRQn);
}

void Timer14_Init(void)
{
	// Pulse pin: PE9
	// Enable pin: PF13
	// Direction pin: PF12
	TIM_TypeDef *pTIM;
	pTIM = TIM14;
	
	RCC->APB1ENR |= (1 << 8);
	RCC->APB1RSTR |= (1 << 8);
	RCC->APB1RSTR &= ~(1 << 8);
	
	pTIM->CR1 = 0;
	//pTIM->ARR = 1000 - 1; // 1000us          freq =  (1/ARR)Hz
	pTIM->ARR = 62 - 1; // freq= 16 KHz
    //pTIM->BDTR = (15 << 1);	// Main output enable
	pTIM->CCER = (1 << 0);
	pTIM->CCMR1 = (0 << 0)|(1 << 3)|(0x7 << 4);
	pTIM->CCR1 = pTIM->ARR/2;
	pTIM->CNT = 0;
	pTIM->CR1 = (0 << 0)|(0 << 1)|(1 << 2)|(0 << 3)|(0 << 4)|(0 << 5)|(0 << 7)|(0 << 8)|(0 << 11);
	//pTIM->DCR = 0;
	pTIM->DIER = (1 << 0); // Update interrupt enabled
	//pTIM->DMAR = 0;
	pTIM->EGR = (1 << 0);		// Event generation
	pTIM->OR = 0;
	//pTIM->PSC = 108 - 1;
	pTIM->PSC = 432 - 1;
    //pTIM->RCR = 0;
	//pTIM->SMCR = 0;
	pTIM->SR = 0xFFFF;
	
	pTIM->CR1 |= (1 << 0);

    NVIC_ClearPendingIRQ (TIM8_TRG_COM_TIM14_IRQn);
    //NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 4, 1);
    NVIC_SetVector(TIM8_TRG_COM_TIM14_IRQn, (uint32_t)TIM8_TRG_COM_TIM14_IRQHandler_Auxiliar);
    NVIC_EnableIRQ (TIM8_TRG_COM_TIM14_IRQn);
}

// Configuracion de timer para encoders
void Timer1_Init(void)
{
	TIM_TypeDef *pTIM;
	pTIM = TIM1;
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
	
	pTIM->ARR = 65536 - 1; // 1000us//65536 
	pTIM->BDTR = 0;	// Main output enable
	pTIM->CCER = (0 << 0)|(0 << 4)|(0 << 8)|(0 << 12)|(0 << 16)|(0 << 20);
	pTIM->CCMR1 = (0 << 0)|(1 << 3)|(0x7 << 4)|(0 << 8)|(1 << 11)|(0x7 << 12);
	pTIM->CCMR2 = (0 << 0)|(1 << 3)|(0x7 << 4)|(0 << 8)|(1 << 11)|(0x7 << 12);
	pTIM->CCMR3 = 0;
	pTIM->CCR1 = (1 << 15);
	pTIM->CCR2 = (1 << 15);
	pTIM->CCR3 = (1 << 15);
	pTIM->CCR4 = 0;
	pTIM->CCR5 = 0;
	pTIM->CCR6 = 0;
	pTIM->CNT = (1 << 15);
	pTIM->CR1 = (0 << 0)|(0 << 1)|(1 << 2)|(0 << 3)|(0 << 4)|(0 << 5)|(0 << 7)|(0 << 8)|(0 << 11);
	pTIM->CR2 = (0 << 0)|(0 << 2)|(1 << 3)|(0 << 4)|(0 << 7)|(0 << 8)|(0 << 9)|(0 << 10)|(0 << 11)
                |(0 << 12)|(0 << 13)|(0 << 14)|(0 << 16)|(0 << 18)|(0 << 20);
	pTIM->DCR = 0;
	pTIM->DIER = 0;
	pTIM->DMAR = 0;
	pTIM->EGR = 0;
	pTIM->OR = 0;
	pTIM->PSC = 0;
	pTIM->RCR = 0;
	pTIM->SMCR = (0x3 << 0)|(0 << 4)|(0 << 7)|(0 << 8)|(0 << 12)|(0 << 14)|(0 << 15);
	pTIM->SR = 0;
	
  pTIM->CR1 |= (1 << 0);
}

void Timer8_Init(void)
{
	TIM_TypeDef *pTIM;
	pTIM = TIM8;
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
	
	pTIM->ARR = 65536 - 1; // 1000us
    //pTIM->ARR = 4096 - 1; // 1000us
	pTIM->BDTR = 0;	// Main output enable
	pTIM->CCER = (0 << 0)|(0 << 4)|(0 << 8)|(0 << 12)|(0 << 16)|(0 << 20);
	pTIM->CCMR1 = (0 << 0)|(1 << 3)|(0x7 << 4)|(0 << 8)|(1 << 11)|(0x7 << 12);
	pTIM->CCMR2 = (0 << 0)|(1 << 3)|(0x7 << 4)|(0 << 8)|(1 << 11)|(0x7 << 12);
	pTIM->CCMR3 = 0;
	pTIM->CCR1 = (1 << 15);
	pTIM->CCR2 = (1 << 15);
	pTIM->CCR3 = (1 << 15);
	pTIM->CCR4 = 0;
	pTIM->CCR5 = 0;
	pTIM->CCR6 = 0;
	pTIM->CNT = (1 << 15);
	pTIM->CR1 = (0 << 0)|(0 << 1)|(1 << 2)|(0 << 3)|(0 << 4)|(0 << 5)|(0 << 7)|(0 << 8)|(0 << 11);
	pTIM->CR2 = (0 << 0)|(0 << 2)|(1 << 3)|(0 << 4)|(0 << 7)|(0 << 8)|(0 << 9)|(0 << 10)|(0 << 11)
                |(0 << 12)|(0 << 13)|(0 << 14)|(0 << 16)|(0 << 18)|(0 << 20);
	pTIM->DCR = 0;
	pTIM->DIER = 0;
	pTIM->DMAR = 0;
	pTIM->EGR = 0;
	pTIM->OR = 0;
	pTIM->PSC = 0;
	pTIM->RCR = 0;
	pTIM->SMCR = (0x3 << 0)|(0 << 4)|(0 << 7)|(0 << 8)|(0 << 12)|(0 << 14)|(0 << 15);
	pTIM->SR = 0;
	
  pTIM->CR1 |= (1 << 0);
}

void TIM1_TRG_COM_TIM11_IRQHandler_Auxiliar(void)
{
	TIM11->SR = 0x0;
	posicion_actual_insp = TIM1->CNT;
   

	if(m_Driver1_CMD == DRIVER_CONTROL_ON)
	{
        if(TIM1->CNT > ((1 << 15) + m_SetPoint_1 + 100)){
            TIM11->PSC = 216 - 1;
        }else if(TIM1->CNT < ((1 << 15) + m_SetPoint_1 - 100)){
            TIM11->PSC = 216 - 1;
        }else{
            TIM11->PSC = 6912 - 1;
        }

		if(TIM1->CNT > ((1 << 15) + m_SetPoint_1 + 20))
		{
			// Cambiar de direccion antihorario
			GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 0);
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 1);
		}
		else if(TIM1->CNT < ((1 << 15) + m_SetPoint_1 - 20))
		{
			// Cambiar de direccion horario
			GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 1);
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 1);
		}
		else
		{
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 0);
		}
	}
	else if(m_Driver1_CMD == DRIVER_CONTROL_INC)
	{
		GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 1);
		GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
	}
	else if(m_Driver1_CMD == DRIVER_CONTROL_DEC)
	{
		GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 0);
		
		if(GPIO_PIN_ReadState(IO_DRIVE1_IND_PORT, IO_DRIVE1_IND_PIN) == 1)	// inductivo detectado
		{
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 1);
			m_Motor1_MinPos = 1;
		}
		else
		{
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
			m_Motor1_MinPos = 0;
		}
	}
	else if(m_Driver1_CMD == DRIVER_CONTROL_OFF)
	{
		GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 0);
		GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 1);
	}
    else {
        GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 1);
    }
}

void TIM8_TRG_COM_TIM14_IRQHandler_Auxiliar(void)
{
	TIM14->SR = 0x0;
	posicion_actual_exp = TIM8->CNT;

	if(m_Driver2_CMD == DRIVER_CONTROL_ON)
	{

        if(TIM8->CNT > ((1 << 15) + m_SetPoint_2 + 100)){
            TIM14->PSC = 108 - 1;
        }else if(TIM8->CNT < ((1 << 15) + m_SetPoint_2 - 100)){
            TIM14->PSC = 108 - 1;
        }else{
            TIM14->PSC = 432 - 1;
        }

		if(TIM8->CNT > ((1 << 15) + m_SetPoint_2 + 20))
		{
		// Cambiar de direccion antihorario
			GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 0);
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            //GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 1);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 1);
		}
		else if(TIM8->CNT < ((1 << 15) + m_SetPoint_2 - 20))
		{
			// Cambiar de direccion horario
			GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 1);
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            //GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 1);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 1);
		}
		else
		{
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            //GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 1);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 0);
		}
	}
	else if(m_Driver2_CMD == DRIVER_CONTROL_INC)
	{
		GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 1);
		GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
        GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 1);
	}
	else if(m_Driver2_CMD == DRIVER_CONTROL_DEC)
	{
		GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 0);

        if(GPIO_PIN_ReadState(IO_DRIVE2_IND_PORT, IO_DRIVE2_IND_PIN) == 1)	// inductivo detectado
		{
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 1);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 0);
			m_Motor2_MinPos = 1;
		}
		else
		{
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 1);
			m_Motor2_MinPos = 0;
		}
	}
	else if(m_Driver2_CMD == DRIVER_CONTROL_OFF)
	{
		GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 0);
		GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 1);
        GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 0);
	}
    else {
        GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 1);
        GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 0);
    }
}


void Motor1_Initialize(void)
{
	// Driver 1: BRAKE, ENA, DIR
    GPIO_PIN_Init(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, GPIO_MODE_OUT| GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
	GPIO_PIN_Init(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, GPIO_MODE_OUT | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
	GPIO_PIN_Init(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, GPIO_MODE_OUT | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
    GPIO_PIN_Init(IO_DRIVE1_IND_PORT, IO_DRIVE1_IND_PIN, GPIO_MODE_IN | GPIO_PUPD_PU, 0);
    
	// DRIVER 1: CLK - TIM11 -> PWM mode 2 - PF7
	GPIO_PIN_Init(IO_DRIVE1_CLK_PORT, IO_DRIVE1_CLK_PIN, GPIO_MODE_AFX | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_AFX_3, 0);

	// TIM1 -> ENCODER
	// PE9  (AF1), PE11 (AF1)
	GPIO_PIN_Init(IO_ENCODER1_CH1_PORT, IO_ENCODER1_CH1_PIN, GPIO_MODE_AFX | GPIO_PUPD_NONE | GPIO_AFX_1, 0);
	GPIO_PIN_Init(IO_ENCODER1_CH2_PORT, IO_ENCODER1_CH2_PIN, GPIO_MODE_AFX | GPIO_PUPD_NONE | GPIO_AFX_1, 0);
	//GPIO_PIN_Init(IO_ENCODER1_CH1_PORT, IO_ENCODER1_CH1_PIN, GPIO_MODE_AFX | GPIO_PUPD_PU | GPIO_AFX_1, 0);
	//GPIO_PIN_Init(IO_ENCODER1_CH2_PORT, IO_ENCODER1_CH2_PIN, GPIO_MODE_AFX | GPIO_PUPD_PU | GPIO_AFX_1, 0);
 
	// PWM
	Timer11_Init();
	
	// Encoder
	Timer1_Init();
}

void Motor2_Initialize(void)
{
	// Driver 2: BRAKE, ENA, DIR
    GPIO_PIN_Init(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, GPIO_MODE_OUT| GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
	GPIO_PIN_Init(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, GPIO_MODE_OUT | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
	GPIO_PIN_Init(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, GPIO_MODE_OUT | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
	GPIO_PIN_Init(IO_DRIVE2_IND_PORT, IO_DRIVE2_IND_PIN, GPIO_MODE_IN | GPIO_PUPD_PU, 0);

	// DRIVER 2 CLK - TIM14 -> PWM mode 2 - PF9
	GPIO_PIN_Init(IO_DRIVE2_CLK_PORT, IO_DRIVE2_CLK_PIN, GPIO_MODE_AFX | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_AFX_9, 0);

	// TIM8 -> ENCODER
	// PC6  (AF3), PC7 (AF3)
	GPIO_PIN_Init(IO_ENCODER2_CH1_PORT, IO_ENCODER2_CH1_PIN, GPIO_MODE_AFX | GPIO_PUPD_NONE | GPIO_AFX_3, 0);
	GPIO_PIN_Init(IO_ENCODER2_CH2_PORT, IO_ENCODER2_CH2_PIN, GPIO_MODE_AFX | GPIO_PUPD_NONE | GPIO_AFX_3, 0);
	

	// PWM
	Timer14_Init();
        
	
	// Encoder
	Timer8_Init();
}

void Motor1_SetAsOrigen(void)
{
  m_Motor1_MinPos = 0;
	m_Driver1_CMD = DRIVER_CONTROL_DEC;
	while(m_Motor1_MinPos == 0)
		osDelay(10);

	TIM1->CR1 &= ~(1 << 0);
	TIM1->CNT = (1 << 15);
	TIM1->CR1 |= (1 << 0);
}

void Motor2_SetAsOrigen(void)
{
   m_Motor2_MinPos = 0;
	m_Driver2_CMD = DRIVER_CONTROL_DEC;
	while(m_Motor2_MinPos == 0)
		osDelay(10);

	TIM8->CR1 &= ~(1 << 0);
	TIM8->CNT = (1 << 15);
	TIM8->CR1 |= (1 << 0);
}



void Motor1_Write_Command(uint8_t command){
    m_Driver1_CMD = command;
}


void Motor1_Write_Setpoint(int16_t setpoint){
	m_SetPoint_1 = setpoint;
}


void Motor2_Write_Command(uint8_t command){
	m_Driver2_CMD = command;
}


void Motor2_Write_Setpoint(int16_t setpoint){
    m_SetPoint_2 = setpoint;    
}