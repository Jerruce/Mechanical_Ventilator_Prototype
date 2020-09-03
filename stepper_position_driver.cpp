

/* File inclusion */
#include "mbed.h"
#include "project_defines.h"
#include "Driver_GPIO.h"
#include "stepper_position_driver.h"



/* Variable definition */

extern volatile int32_t posicion_actual_insp;
extern volatile int32_t posicion_actual_exp;

volatile int m_SetPoint_1;
volatile int m_SetPoint_2;

volatile int m_Driver1_CMD = DRIVER_CONTROL_OFF;
volatile int m_Driver2_CMD = DRIVER_CONTROL_OFF;

volatile int m_Motor1_MinPos = 0;
volatile int m_Motor2_MinPos = 0;

volatile int16_t motor1_step_counter = 0;
volatile int16_t motor2_step_counter = 0;

/* Function declaration */
void Timer1_Init(void);
void Timer11_Init(void);
void Timer8_Init(void);
void Timer14_Init(void);

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



void TIM1_TRG_COM_TIM11_IRQHandler_Auxiliar(void)
{

    uint8_t dir_state, en_state, brake_state;

	TIM11->SR = 0x0;
   
    dir_state = GPIO_PIN_ReadState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN);
    en_state = GPIO_PIN_ReadState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN);
    brake_state = GPIO_PIN_ReadState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN);

    if((!en_state) && brake_state){
        if(dir_state == 1){
            motor1_step_counter++;
        }else{
            motor1_step_counter--;
        }
    }

    posicion_actual_insp = motor1_step_counter;

	if(m_Driver1_CMD == DRIVER_CONTROL_ON)
	{
        if(motor1_step_counter > (m_SetPoint_1 + INSPIRATION_NEEDLE_VALVE_SPEED_SWITCH_THRESHOLD)){
            TIM11->PSC = 216 - 1;
        }else if(motor1_step_counter < (m_SetPoint_1 - INSPIRATION_NEEDLE_VALVE_SPEED_SWITCH_THRESHOLD)){
            TIM11->PSC = 216 - 1;
        }else{
            //TIM11->PSC = 6912 - 1;
            TIM11->PSC = 864 - 1;
        }

		if(motor1_step_counter  > m_SetPoint_1)
		{
			// Cambiar de direccion antihorario
			GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 0);
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 1);
		}
		else if(motor1_step_counter  < m_SetPoint_1)
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
        GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 1);
	}
	else if(m_Driver1_CMD == DRIVER_CONTROL_DEC)
	{
        TIM11->PSC = 864 - 1;

		GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 0);
		
		if(GPIO_PIN_ReadState(IO_DRIVE1_IND_PORT, IO_DRIVE1_IND_PIN) == 1)	// inductivo detectado
		{
            GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 0);
			m_Motor1_MinPos = 1;
            motor1_step_counter = 0;
		}
		else
		{
			GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 1);
			m_Motor1_MinPos = 0;
		}
	}
	else if(m_Driver1_CMD == DRIVER_CONTROL_OFF)
	{
		GPIO_PIN_SetState(IO_DRIVE1_DIR_PORT, IO_DRIVE1_DIR_PIN, 0);
		GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 1);
        GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 0);
	}
    else {
        GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 1);
        GPIO_PIN_SetState(IO_DRIVE1_BRAKE_PORT, IO_DRIVE1_BRAKE_PIN, 0);
    }
}



void TIM8_TRG_COM_TIM14_IRQHandler_Auxiliar(void)
{

    uint8_t dir_state, en_state, brake_state;

    TIM14->SR = 0x0;

    dir_state = GPIO_PIN_ReadState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN);
    en_state = GPIO_PIN_ReadState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN);
    brake_state = GPIO_PIN_ReadState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN);

    if((!en_state) && brake_state){
        if(dir_state == 1){
            motor2_step_counter++;
        }else{
            motor2_step_counter--;
        }
    }
	
    posicion_actual_exp = motor2_step_counter;
    

	if(m_Driver2_CMD == DRIVER_CONTROL_ON)
	{

        if(motor2_step_counter > (m_SetPoint_2 + EXPIRATION_BALL_VALVE_SPEED_SWITCH_THRESHOLD)){
            TIM14->PSC = 108 - 1;
        }else if(motor2_step_counter < (m_SetPoint_2 - EXPIRATION_BALL_VALVE_SPEED_SWITCH_THRESHOLD)){
            TIM14->PSC = 108 - 1;
        }else{
            TIM14->PSC = 432 - 1;
        }

		if(motor2_step_counter > m_SetPoint_2)
		{
		// Cambiar de direccion antihorario
			GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 0);
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 1);
		}
		else if(motor2_step_counter < m_SetPoint_2)
		{
			// Cambiar de direccion horario
			GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 1);
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 1);
		}
		else
		{
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
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

        // Set speed for going home
        TIM14->PSC = 432 - 1;

		GPIO_PIN_SetState(IO_DRIVE2_DIR_PORT, IO_DRIVE2_DIR_PIN, 0);

        if(GPIO_PIN_ReadState(IO_DRIVE2_IND_PORT, IO_DRIVE2_IND_PIN) == 1)	// inductivo detectado
		{
			GPIO_PIN_SetState(IO_DRIVE2_ENA_PORT, IO_DRIVE2_ENA_PIN, 0);
            GPIO_PIN_SetState(IO_DRIVE2_BRAKE_PORT, IO_DRIVE2_BRAKE_PIN, 0);
			m_Motor2_MinPos = 1;
            motor2_step_counter = 0;
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

	GPIO_PIN_Init(IO_ENCODER1_CH1_PORT, IO_ENCODER1_CH1_PIN, GPIO_MODE_AFX | GPIO_PUPD_NONE | GPIO_AFX_1, 0);
	GPIO_PIN_Init(IO_ENCODER1_CH2_PORT, IO_ENCODER1_CH2_PIN, GPIO_MODE_AFX | GPIO_PUPD_NONE | GPIO_AFX_1, 0);

	// PWM
	Timer11_Init();
	
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
        
}


void Motor1_SetAsOrigen(void)
{
    m_Motor1_MinPos = 0;
	m_Driver1_CMD = DRIVER_CONTROL_DEC;

	while(m_Motor1_MinPos == 0){
        //Wait until motor reaches the inductive sensor
    }
    motor1_step_counter = 0;

	osDelay(10);
}


void Motor2_SetAsOrigen(void)
{
    m_Motor2_MinPos = 0;
	m_Driver2_CMD = DRIVER_CONTROL_DEC;

	while(m_Motor2_MinPos == 0){
        //Wait until motor reaches the inductive sensor
    }
    motor2_step_counter = 0;

	osDelay(10);
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