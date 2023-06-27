
 /******************************************************************************
 *
 * Module: Ultrasonic
 *
 * File Name: UltraSonic.h
 *
 * Description: Header file for the UltraSonic driver for STM
 *
 * Author: Omar Shamekh
 *
 *******************************************************************************/
#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

/******************* Ultrasonic HW Ports and Pins Defination*******************/

/*************** Right UltraSonic *****************/
// Echo is connected to E9 which has the timer 1 channel we are going to use
#define Right_ECHO_PORT_ID GPIOE
#define Right_ECHO_PIN_ID GPIO_PIN_9

// TRIG pin is a normal output GPIO pin
#define Right_TRIG_PORT_ID GPIOE
#define Right_TRIG_PIN_ID GPIO_PIN_10

/*************** Left UltraSonic *****************/
#define Left_ECHO_PORT_ID GPIOE
#define Left_ECHO_PIN_ID GPIO_PIN_11

#define Left_TRIG_PORT_ID GPIOE
#define Left_TRIG_PIN_ID GPIO_PIN_12
/*************** Rear UltraSonic *****************/
#define Rear_ECHO_PORT_ID GPIOE
#define Rear_ECHO_PIN_ID GPIO_PIN_13

#define Rear_TRIG_PORT_ID GPIOE
#define Rear_TRIG_PIN_ID GPIO_PIN_14


//Trigger Pulse duration is set to be 10 us in order to generate 8 sound pulses
#define TRIG_PulseDuration 10

#include <stdint.h>
#include "stm32f4xx_hal.h"
//#include "FreeRTOS.h"
//#include "queue.h"
//extern xQueueHandle xDistanceQueue;
/**************************  Functions Prototypes  *****************************/

/* This function used in order to generate a time delay in micro-seconds  */
void delay_us(uint32_t time);


/* This function is used to get distance measured */
void UltraSonics_ReadAll (void); //Activate 3 Sides
void UltraSonic_Read_Left (void);
void UltraSonic_Read_Right (void);
void UltraSonic_Read_Rear (void);

uint32_t Distance_Covered(void);
#endif /* INC_ULTRASONIC_H_ */
