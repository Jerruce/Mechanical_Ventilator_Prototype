/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "stdint.h"
#include "stdio.h"
#include "rtos.h"
#include "mcu.h"
#include "project_defines.h"
#include "global.h"
#include "Driver_USART.h"
#include "Driver_GPIO.h"
#include "Task_GUI.h"
#include "proximal_flow_sensor.h"
#include "proximal_pressure_sensor.h"
#include "filters.h"
#include "electric_valves.h"
#include "pid_basic.h"
#include "breath_sequence.h"
#include "FiO2_sensor.h"
#include "Filter.h"
#include "ventilator_alarm.h"
#include "fuzzy_controllers_basic.h"
#include "stepper_position_driver.h"

//#include "AppDefinition.h"

extern int m_SetPoint_2;
// Definicion de pines
// LED PB7 (prueba)


#define miled PE_8;


volatile int32_t posicion_actual_insp = 0;
volatile int32_t posicion_actual_exp = 0;
Breath_Mode_t modo_ventilacion= VC_CMV_MODE;
volatile uint8_t alarm_enable= 1, ventilator_start_stop= 1;//torres
volatile float frec_resp_sp=20.0,  t_exp_sp=2.0, volumen_tidal_sp=200, peep_sp=0.0, pip_sp=10.0;//garcía
static float Volumen_peak, flow_peak, pressure_peak;
volatile float valor_volumen_promedio, valor_error_volumen;


KALMAN_PARAM myFILTER;

AppStatus_Type m_AppStatus;
AppSetting_Type m_AppSetting;

// Tareas
void Task_Control(void);
void Task_GUI(void);
void Task_TCPServer(void);

Thread threadId_Control;
Thread threadId_GUI;
Thread threadId_TCPServer;
// Hilos para leer sensores e imprimir valores
Thread threadId_SensorRead;
Thread threadId_SensorDisplay;
Thread threadId_Timers;
Thread threadId_Expiration_On_Off_Valve;
Thread threadId_Flag_Manager;
Thread threadId_Pruebita;


// Funciones de librerias

void Mapeado(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;
}

// Entrega los caracteres del valor de una variable tipo Word
int Word2StrMSB0(uint32_t Word, char *Buffer, int Digits)
{
	int pX;
	
	for(pX=0; pX<Digits; pX++)
	{
		Buffer[Digits-pX-1] = (Word % 10) + 48;
		Word = Word / 10;
	}
	
	return Digits;
}
 
// Driver CLK


void IO_Initialize(void)
{
	// LED
	GPIO_PIN_Init(IO_LED_PORT, IO_LED_PIN, GPIO_MODE_OUT | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_PUPD_NONE, 0);
	
	// Pulsador
	GPIO_PIN_Init(IO_BUTTON_PORT, IO_BUTTON_PIN, GPIO_MODE_IN | GPIO_PUPD_PD, 0);
}

void LED_State(int State)
{
	GPIO_PIN_SetState(IO_LED_PORT, IO_LED_PIN, State);
}

void USART_Initialize(void)
{
	UART_Config_Type UsartConfig;
	
	// USART2 -> DISPLAY
	// PD5-TXD  (AF7), PD6-RXD  (AF7)
	GPIO_PIN_Init(GPIOD, 5, GPIO_MODE_AFX | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_AFX_7, 1);
	GPIO_PIN_Init(GPIOD, 6, GPIO_MODE_AFX | GPIO_PUPD_PU | GPIO_AFX_7, 0);


    // USART3 -> DEBUG
	// PD8-TXD  (AF7), PD9-RXD  (AF7)
	GPIO_PIN_Init(GPIOD, 8, GPIO_MODE_AFX | GPIO_OTYPE_PP | GPIO_OSPEED_MEDIUM | GPIO_AFX_7, 1);
	GPIO_PIN_Init(GPIOD, 9, GPIO_MODE_AFX | GPIO_PUPD_PU | GPIO_AFX_7, 0);    
	
	// UARTs Init
	UART_GetDefaultSetting(&UsartConfig);

	UsartConfig.Baudrate = 250000;
	UsartConfig.Parity = UART_PARITY_NONE;
	
	UsartConfig.UartId = 2;
	UART_Init(&UsartConfig);

    UsartConfig.Baudrate = 115200;
	UsartConfig.Parity = UART_PARITY_NONE;
	
	UsartConfig.UartId = 3;
	UART_Init(&UsartConfig);
}

void Task_Control(void)
{
	int vButtonLastState;
	int vButtonCurState;
	int vKeyState;
	int vKeyLastState;
	
	Mapeado();
	IO_Initialize();
	
    Motor1_Write_Setpoint(0);
    Motor2_Write_Setpoint(0);
    Motor1_Write_Command(DRIVER_CONTROL_OFF);
    Motor2_Write_Command(DRIVER_CONTROL_OFF);

	// Inicializa Periferico
	//Motor1_Initialize();
	Motor2_Initialize();
	
	// Establece posicion actual como punto de referencia
	//Motor1_SetAsOrigen();
    //GPIO_PIN_SetState(IO_LED_PORT, IO_LED_PIN, 1);
	Motor2_SetAsOrigen();
    //GPIO_PIN_SetState(IO_LED_PORT, IO_LED_PIN, 0);
    //Electric_Valves_Initialize();
    
    //Motor1_Write_Command(DRIVER_CONTROL_ON);
    Motor2_Write_Command(DRIVER_CONTROL_ON);

	vButtonCurState = GPIO_PIN_ReadState(IO_BUTTON_PORT, IO_BUTTON_PIN);
	vButtonLastState = vButtonCurState;
	vKeyState = 0;

	while (1)
	{
		//LED_State(2);
		
		vButtonCurState = GPIO_PIN_ReadState(IO_BUTTON_PORT, IO_BUTTON_PIN);
		
		if(vButtonLastState == vButtonCurState)
		{
			if(vButtonCurState != 0)
				vKeyState = 1;
			else
				vKeyState = 0;
		}
		vButtonLastState = vButtonCurState;
	
		// Para probar control
       // GPIO_PIN_SetState(IO_DRIVE1_ENA_PORT, IO_DRIVE1_ENA_PIN, 0);
      //  m_Driver1_CMD = DRIVER_CONTROL_ON;
	/* 	if((vKeyLastState != vKeyState) && (vKeyState != 0))
		{
			m_Driver1_CMD++;
			m_Driver2_CMD++;
			
			if(m_Driver1_CMD > DRIVER_CONTROL_ON)
				m_Driver1_CMD = DRIVER_CONTROL_OFF;
			
			if(m_Driver2_CMD > DRIVER_CONTROL_ON)
				m_Driver2_CMD = DRIVER_CONTROL_OFF;
        }*/
		
		vKeyLastState = vKeyState;
		
		osDelay(50);
	}
}

char COMM_BUFFER_TX[100];
int vLength;

void Task_Test(void)
{
	USART_Initialize();
	while (1)
	{
		vLength = sprintf(COMM_BUFFER_TX, "Registro: ");
		vLength += Word2StrMSB0(TIM8->CNT, COMM_BUFFER_TX + vLength, 8);
		vLength += sprintf(COMM_BUFFER_TX + vLength, " \r\n");
		
	   // UART3_SendData((uint8_t *)COMM_BUFFER_TX, vLength);
		osDelay(1000);
	}
}

// <dato>
volatile uint16_t number = 0; 
volatile uint16_t motor_position = 0;
char buffer_aux[20];

volatile uint8_t command;
volatile uint8_t rx_buffer[20];
volatile float valor_kp,valor_ki;
volatile float kp_recibido = 0.0,  ki_recibido= 0, kd_recibido= 0;
volatile float setpoint_recibido = 0;
volatile float setpoint_flujo;

char Comm_RxBuffer[32];
int _rxLevel;
void Task_TCPServer(void)
{

    static float acumulador = 0;
    static float factor = 1.0;
    static uint8_t fracc  = 0;

    //char vChar;
    //_rxLevel = 0;

    while (1) {
        while(UART3_IsAvailable() != 0)
        {
            command = UART3_ReadByte();

            if(command == '.'){
                fracc = 1;
            }else if(command == 'p'){
                kp_recibido = acumulador;
                acumulador = 0;
                factor = 1.0;
                fracc = 0,
                system_flags |= (1 << COMMAND_RECEIVED_DATA_FLAG);
            }else if(command == 'i'){
                ki_recibido = acumulador;
                acumulador = 0;
                factor = 1.0;
                fracc = 0,
                system_flags |= (1 << COMMAND_RECEIVED_DATA_FLAG);
            }else if(command == 'd'){
                kd_recibido = acumulador;
                acumulador = 0;
                factor = 1.0;
                fracc = 0,
                system_flags |= (1 << COMMAND_RECEIVED_DATA_FLAG);
            }else if(command == 's'){
                setpoint_recibido = acumulador;
                acumulador = 0;
                factor = 1.0;
                fracc = 0,
                system_flags |= (1 << COMMAND_RECEIVED_DATA_FLAG);
            }else{
                if(fracc){
                    factor /= 10.0;
                    acumulador = acumulador + ((command - 48)*factor);
                }else{
                    acumulador = (acumulador * 10) + ((command - 48)*factor);
                }
                
            }
        }

        osDelay(10);
    }
}


volatile uint32_t system_flags;
float flujo, presion, volumen, altura_columna_agua;
float flujo_pid;
int32_t flujo_entero, presion_entero;
int32_t cociente, residuo;
char buffer_sensores[50];


void Task_SensorRead(void){
    int vContador =0;
    Proximal_Pressure_Sensor_Initialize();
    Proximal_Flow_Sensor_Initialize();
    Filters_Initialize();
    while(1){
        /* Read the sensors */
        Proximal_Pressure_Sensor_Read();
        presion = Proximal_Pressure_Sensor_Get_Pressure();
        Proximal_Flow_Sensor_Read();
        flujo = Proximal_Flow_Sensor_Get_Flow();
        FiO2_Sensor_Read();
        /* Filter the measurements */
        Apply_Flow_LPF_Filter(flujo);
        Apply_Pressure_LPF_Filter(presion);
        /* Read the filtered measurements */
        presion = Get_Pressure_LPF_Output();
        flujo = Get_Flow_LPF_Output();
        //Apply_Flow_PID_LPF_Filter(flujo);
        //flujo_pid = Get_Flow_PID_LPF_Output();
        flujo_pid = Filtering_Kalman(&myFILTER, flujo);
        Proximal_Flow_Sensor_Calculate_Volume(flujo);
        volumen = Proximal_Flow_Sensor_Get_Volume();
        //flujo = vContador++;
        if(vContador > 100)
            vContador = 0;
        InsertPoints(&flujo, &presion, &volumen); 
        osDelay(10);
    }

      
}

float Signal1 =0;

void Task_SensorDisplay(void){

    static int32_t display_posicion;
    static uint16_t fio2_entero = 0;
    static uint16_t conta_10ms = 0;
    static uint16_t conta_100ms = 0;
    static uint16_t presion_anterior = 0;   
    int32_t cambio_presion = 0;
    int32_t flujo_anterior = 0;
    int32_t cambio_flujo = 0;
    static uint8_t contador_10ms = 0;

    int vEncoder =0;
    
    float Signal2 =0;
    Kalman_init(&myFILTER);

    while(1){
        //display_posicion = posicion_actual_insp;
        display_posicion = posicion_actual_exp;
        presion_entero = (int32_t)(presion * 100.00);
        //sprintf(buffer_sensores, "presion = %d\n\r", presion_entero);
       // UART3_SendString((uint8_t *)buffer_sensores); while(UART3_TxLevel() > 0) osDelay(1); osDelay(1);
        flujo_entero = (int32_t)(flujo * 100);
        //sprintf(buffer_sensores, "flujo = %d\n\n\r", flujo_entero);
          /* Imprime flujo medido y setpoint (todo multiplicado por 100) en el serial plotter */
        //flujo_entero = (int32_t)(flujo_pid*100.00);
                //sprintf(buffer_sensores, "%d,%d,%d\r\n", (int32_t)(pip_sp * 100), presion_entero, m_SetPoint_1 * 10
                        //(TIM1->CNT - (1 << 15))*10,
                        //(int32_t)(flujo * 100.0)
                         //);
        //Signal1 = Filtering_Kalman(&myFILTER, flujo);
        //Signal2 = Get_Flow_PID_LPF_Output();
        //vEncoder = (int32_t)((TIM1->CNT) - (1 << 15))*10;
        //sprintf(buffer_sensores, "%d,%d\r\n", (int32_t)(setpoint_recibido * 100), presion_entero);
        //sprintf(buffer_sensores, "%d,%d\r\n", (int32_t)(Breath_System_Get_Flow_Setpoint() * 100), flujo_entero);
        //sprintf(buffer_sensores, "%d,%d,%d\r\n", (int32_t)(setpoint_recibido * 100), flujo_entero,cambio_flujo);
        //sprintf(buffer_sensores, "%d,%d,%d\r\n", (int32_t)(setpoint_recibido * 100), presion_entero,cambio_presion);
        //sprintf(buffer_sensores, "%d,%d\r\n", (int32_t)(setpoint_recibido), display_posicion - 32700);
        sprintf(buffer_sensores, "%d,%d\r\n", (int32_t)(m_SetPoint_2), display_posicion);
        //sprintf(buffer_sensores, "%d\r\n", flujo_entero);
        //sprintf(buffer_sensores, "%d,%d,%d,%d,%d\r\n", (int32_t)(setpoint_recibido * 100), flujo_entero, m_SetPoint_1 * 10,
       //                  vEncoder,
       //                  (int)(Signal1*100) + 1000
                         //(int)(Signal2*100) + 1000
        //                 );
        //sprintf(buffer_sensores, "%d,%d,%d\r\n", (int32_t)(setpoint_recibido * 100), presion_entero, m_SetPoint_2 * 10);
        //sprintf(buffer_sensores, "%d,%d,%d\r\n", (int32_t)(peep_sp * 100), presion_entero, m_SetPoint_2 * 10);
        //sprintf(buffer_sensores, "%d,%d,%d\r\n", (int32_t)(peep_sp * 100), presion_entero, flujo_entero);
        //sprintf(buffer_sensores, "%d,%d\r\n", (int32_t)(altura_columna_agua * 100), (int32_t)(peep_sp * 100));
        //sprintf(buffer_sensores, "%d,%d\r\n", (int32_t)(setpoint_recibido), m_SetPoint_2);

       /*  if(modo_ventilacion == VC_CMV_MODE){
            sprintf(buffer_sensores, "VC_CMV\n\r");
        }else if(modo_ventilacion == PC_CMV_MODE){
            sprintf(buffer_sensores, "PC_CMV\n\r");
        }else{
            sprintf(buffer_sensores, "N.D.\n\r");
        }
*/

        //sprintf(buffer_sensores, "Modo: %d\n\r", m_AppSetting.ApplicationVar.ControlMode);
        //sprintf(buffer_sensores, "%d\r\n", (uint16_t)(10*Get_FiO2_Value()));
        UART3_SendString((uint8_t *)buffer_sensores); while(UART3_TxLevel() > 0) osDelay(1); osDelay(1);

        conta_10ms++;

        contador_10ms++;

        if(contador_10ms >= 5){
            contador_10ms = 0;

            cambio_presion = presion_entero - presion_anterior;
            presion_anterior = presion_entero;
            cambio_flujo = flujo_entero - flujo_anterior;
            flujo_anterior = flujo_entero;
/* 
            if(cambio_presion > 50){
                cambio_presion = 50;
            }else if(cambio_presion < -100){
                cambio_presion = -100;
            }else{
                //does nothing
            }
*/
            //sprintf(buffer_sensores, "%d\r\n", cambio_presion);
            //UART3_SendString((uint8_t *)buffer_sensores); while(UART3_TxLevel() > 0) osDelay(1); osDelay(1);
 
        }
 

        if(conta_10ms >= 10){
            conta_10ms = 0;

            flow_peak = Breath_System_Get_Flow_Indicator_Value(); 
            pressure_peak = Breath_System_Get_Pressure_Indicator_Value();
            Volumen_peak = Breath_System_Get_Volume_Indicator_Value();
            m_AppStatus.FIO2 = (uint16_t)Get_FiO2_Value();//10 * Get_FiO2_Value();
            
            UpdateMaxValues((int)flow_peak, (int)pressure_peak, (int)Volumen_peak);
            //Breath_System_Set_Mode(modo_ventilacion);
            Breath_System_Set_Mode((Breath_Mode_t)m_AppSetting.ApplicationVar.ControlMode);
            PEEP_Setpoint_Update((uint8_t)peep_sp);
            Resp_Frequency_Setpoint_Update((uint8_t)frec_resp_sp);
            I_E_Ratio_Setpoint_Update(1.0, t_exp_sp);
            Tidal_Volume_Setpoint_Update((uint16_t)volumen_tidal_sp);
            //PIP_Setpoint_Update((uint16_t)pip_sp);

        }

        //volatile uint8_t alarm_enable= 1, ventilator_start_stop= 1;

        conta_100ms++;
        if(conta_100ms >= 100)
        {
            conta_100ms = 0;

            //fio2_entero = (uint16_t)Get_FiO2_Value();

         //   conta_100ms = 0;
        //    setpoint_flujo = Breath_System_Get_Flow_Setpoint();
            //sprintf(buffer_sensores, "start_stop=%d\r\n", (int32_t)ventilator_start_stop);
           // sprintf(buffer_sensores, "%d p\r\n", fio2_entero);
            //UART3_SendString((uint8_t *)buffer_sensores); while(UART3_TxLevel() > 0) osDelay(1); osDelay(1);
        }


       // flow_peak  =  17;
      // sprintf((char *)buffer_sensores, "volumen = %d\r\n", (int)m_AppSetting.ApplicationVar..);

      //  UART3_SendString((uint8_t *)buffer_sensores); while(UART3_TxLevel() > 0) osDelay(1); osDelay(1);
            //while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);
        osDelay(10);

    }
}


void Task_Flag_Manager(void){
    static uint16_t pid_counter = 0;

    while(1){
        pid_counter++;
        if(pid_counter == 5){
          pid_counter = 0;
          system_flags |= (1 << PID_ACTION_UPDATE_FLAG); 
        }

        Alarm_Time_Count();
        osDelay(10);

    }
   
}


void Task_Pruebita(void){
    //Breath_System_Initialize();
    //Inspiration_Needle_Valve_Go_Home();
    //Inspiration_Needle_Valve_Initialize();
    //Inspiration_Needle_Valve_Go_Home();
    //wait(1);

    while(1){
         
         if(system_flags & (1 << COMMAND_RECEIVED_DATA_FLAG)){                      
            //Inspiration_Flow_PID_Controller_Set_Kp(kp_recibido);
            //PIP_PID_Controller_Set_Kp(kp_recibido);
            //Flow_Fuzzy_Inc_Controller_Set_GE(kp_recibido);
            //Inspiration_Flow_PID_Controller_Set_Ki(ki_recibido);
            //PIP_PID_Controller_Set_Ki(ki_recibido);
            //Flow_Fuzzy_Inc_Controller_Set_GCU(ki_recibido);
            //Inspiration_Flow_PID_Controller_Set_Kd(kd_recibido);
            //PIP_PID_Controller_Set_Kd(kd_recibido);
            //Flow_Fuzzy_Inc_Controller_Set_GCE(kd_recibido);
            //Flow_Setpoint_Update((uint8_t)setpoint_recibido);
            //PEEP_Setpoint_Update((uint8_t)setpoint_recibido);
            //PIP_Setpoint_Update((uint8_t)setpoint_recibido);
            //Inspiration_Needle_Valve_Write_Step_Setpoint(setpoint_recibido);
            //Inspiration_Needle_Valve_Go_To_Setpoint();
            //Expiration_Ball_Valve_Write_Step_Setpoint(setpoint_recibido);
            //Expiration_Ball_Valve_Go_To_Setpoint();
            Motor2_Write_Setpoint((int16_t)setpoint_recibido);
            system_flags &= ~(1 << COMMAND_RECEIVED_DATA_FLAG);
       }
        
        //VC_CMV_State_Machine();//PC_CMV_State_Machine();//Breath_State_Machine();
        //ALARM_BUZZER_ON();
        //Alarm_Buzzer_Update();
        osDelay(1);
    }
   
}


void Init_Threads (void) 
{
    threadId_Control.start(Task_Control);
    threadId_GUI.start(Task_GUI);
    threadId_TCPServer.start(Task_TCPServer);
    threadId_SensorRead.start(Task_SensorRead);
    threadId_SensorDisplay.start(Task_SensorDisplay);
    threadId_Timers.start(Task_Timers);
    threadId_Flag_Manager.start(Task_Flag_Manager);
    threadId_Pruebita.start(Task_Pruebita);
}

int main() {
    Mapeado();
    USART_Initialize();       
    Init_Threads();         
    while (true){
        osDelay(200);
    }
}
