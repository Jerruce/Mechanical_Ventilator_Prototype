

#ifndef STEPPER_POSITION_DRIVER_H_
#define STEPPER_POSITION_DRIVER_H_


/* File inclusion */
#include "stdint.h"


#define IO_LED_PORT GPIOB
#define IO_LED_PIN 	7

// Pulsador PB7 (prueba)
#define IO_BUTTON_PORT          GPIOC
#define IO_BUTTON_PIN 	        13


// DRIVER 1
#define IO_DRIVE1_ENA_PORT      GPIOC                        
#define IO_DRIVE1_ENA_PIN       12

#define IO_DRIVE1_DIR_PORT      GPIOF
#define IO_DRIVE1_DIR_PIN       6

#define IO_DRIVE1_CLK_PORT      GPIOF
#define IO_DRIVE1_CLK_PIN       7

#define IO_DRIVE1_IND_PORT      GPIOD
#define IO_DRIVE1_IND_PIN       2

#define IO_DRIVE1_BRAKE_PORT    GPIOE
#define IO_DRIVE1_BRAKE_PIN     12


// DRIVER 2
#define IO_DRIVE2_ENA_PORT      GPIOF
#define IO_DRIVE2_ENA_PIN       8

#define IO_DRIVE2_DIR_PORT      GPIOG
#define IO_DRIVE2_DIR_PIN       1

#define IO_DRIVE2_CLK_PORT      GPIOF
#define IO_DRIVE2_CLK_PIN       9

#define IO_DRIVE2_IND_PORT      GPIOB
#define IO_DRIVE2_IND_PIN       5

#define IO_DRIVE2_BRAKE_PORT    GPIOE
#define IO_DRIVE2_BRAKE_PIN     10

// ENCODER 1
#define IO_ENCODER1_CH1_PORT    GPIOE
#define IO_ENCODER1_CH1_PIN     9


#define IO_ENCODER1_CH2_PORT    GPIOE
#define IO_ENCODER1_CH2_PIN     11

// ENCODER 2
#define IO_ENCODER2_CH1_PORT    GPIOC
#define IO_ENCODER2_CH1_PIN     6

#define IO_ENCODER2_CH2_PORT GPIOC
#define IO_ENCODER2_CH2_PIN  7

// Definicion de comandos para el motor
#define DRIVER_CONTROL_OFF	0
#define DRIVER_CONTROL_INC	1
#define DRIVER_CONTROL_DEC	2
#define DRIVER_CONTROL_ON   3


/* Function declaration */
void Motor1_Initialize(void);
void Motor2_Initialize(void);
void Motor1_SetAsOrigen(void);
void Motor2_SetAsOrigen(void);
void Motor1_Write_Command(uint8_t command);
void Motor1_Write_Setpoint(int16_t setpoint);
void Motor2_Write_Command(uint8_t command);
void Motor2_Write_Setpoint(int16_t setpoint);

#endif