/******************************************************************************
 *
 * Module: UltraSonic
 *
 * File Name: UltraSonic.c
 *
 * Description: Source file for the UltraSonic for STM
 *
 * Author: Omar Shamekh
 *
 *******************************************************************************/
#include "UltraSonic.h"





/********************** Parameters Definitions  ******************************/

/*IC values 1 and 2 are used to captured the time of the high pulse
 * in order to calculate the Right_Distance traveled
 *  __________
 * |          |
 * |     	  |
 * |<--diff-->|_______________
 * ↑          ↑
 * IC1       IC2
 ******************************************/
/**********        Right Variables            ***********/
uint32_t Right_IC_Val1=0;
uint32_t Right_IC_Val2=0;
uint32_t Right_difference=0;
/* To Check if the 1st IC is captured or not*/
uint8_t Right_First_Captured_Flag = 0;
uint32_t Right_Distance  = 0;

/**********        Left Variables            ***********/
uint32_t Left_IC_Val1=0;
uint32_t Left_IC_Val2=0;
uint32_t Left_difference=0;
/* To Check if the 1st IC is captured or not*/
uint8_t Left_First_Captured_Flag = 0;
uint32_t Left_Distance  = 0;

/**********        Rear Variables            ***********/
uint32_t Rear_IC_Val1=0;
uint32_t Rear_IC_Val2=0;
uint32_t Rear_difference=0;
/* To Check if the 1st IC is captured or not*/
uint8_t Rear_First_Captured_Flag = 0;
uint32_t Rear_Distance  = 0;


extern TIM_HandleTypeDef htim1;


/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/
void delay_us (uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

/*In order to get Right_Distance the ultrasonic needs to set its trig pin high for 10us
 * then set it to low and then check the time taken and calculate Right_Distance with it */
void UltraSonic_Read_Left (void)
{
	HAL_GPIO_WritePin(Left_TRIG_PORT_ID, Left_TRIG_PIN_ID, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(TRIG_PulseDuration);  // wait for 10 us
	HAL_GPIO_WritePin(Left_TRIG_PORT_ID, Left_TRIG_PIN_ID, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
}
void UltraSonic_Read_Right (void)
{
	HAL_GPIO_WritePin(Right_TRIG_PORT_ID, Right_TRIG_PIN_ID, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(TRIG_PulseDuration);  // wait for 10 us
	HAL_GPIO_WritePin(Right_TRIG_PORT_ID, Right_TRIG_PIN_ID, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
void UltraSonic_Read_Rear (void)
{
	HAL_GPIO_WritePin(Rear_TRIG_PORT_ID, Rear_TRIG_PIN_ID, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(TRIG_PulseDuration);  // wait for 10 us
	HAL_GPIO_WritePin(Rear_TRIG_PORT_ID, Rear_TRIG_PIN_ID, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
}

void UltraSonics_ReadAll (void)
{
	UltraSonic_Read_Left ();
	UltraSonic_Read_Right ();
	//UltraSonic_Read_Rear ();
}


/* Call back function is used to determine how the timer will work
 * as it needs to capture the rising edge 1st and stores its time in Right_IC_Val1
 * then polarity is changed in order to capture the falling edge and stores its occurrence time in Right_IC_Val2
 * then their Right_difference is the high pulse time. */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		// To check if the first value is not captured yet
		if (Right_First_Captured_Flag==0)
		{
			// stores the first value
			Right_IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			// set the first captured as true as we now needs to capture the falling edge
			Right_First_Captured_Flag = 1;
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		// to check if the first is already captured
		else if (Right_First_Captured_Flag==1)
		{
			// stores second value
			Right_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			// reset the counter
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (Right_IC_Val2 > Right_IC_Val1)
			{
				Right_difference = Right_IC_Val2-Right_IC_Val1;
			}

			else if (Right_IC_Val1 > Right_IC_Val2)
			{
				// to check for any overflows
				Right_difference = (0xffff - Right_IC_Val1) + Right_IC_Val2;
			}

			/*Then to Calculate the Right_Distance:
			 *
			 * speed of sound in air = 344m/s = 34400 cm/s
			 * total Right_Distance = speed * echo time (diff bet Right_IC_Val1 and Right_IC_Val2)
			 * Right_Distance = total Right_Distance/2 (as the wave goes then returns back)
			 * Right_Distance = 17000*echo time
			 *
			 * as the cpu works with 168MHz freq and prescaler of 168
			 * then to execute 1 second we need 1us
			 *
			 *therefore, distace = 17000*echo time* 1us = 0.017*echotime
			 *
			 */

			Right_Distance = Right_difference * 0.0175;
			Right_First_Captured_Flag = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			// xQueueSendFromISR(xDistanceQueue, &Right_Distance, NULL);
		}
	}
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				// To check if the first value is not captured yet
				if (Left_First_Captured_Flag==0)
				{
					// stores the first value
					Left_IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
					// set the first captured as true as we now needs to capture the falling edge
					Left_First_Captured_Flag = 1;
					// Now change the polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				// to check if the first is already captured
				else if (Left_First_Captured_Flag==1)
				{
					// stores second value
					Left_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
					// reset the counter
					__HAL_TIM_SET_COUNTER(htim, 0);

					if (Left_IC_Val2 > Left_IC_Val1)
					{
						Left_difference = Left_IC_Val2-Left_IC_Val1;
					}

					else if (Left_IC_Val1 > Left_IC_Val2)
					{
						// to check for any overflows
						Left_difference = (0xffff - Left_IC_Val1) + Left_IC_Val2;
					}

					/*Then to Calculate the Left_Distance:
					 *
					 * speed of sound in air = 344m/s = 34400 cm/s
					 * total Left_Distance = speed * echo time (diff bet Left_IC_Val1 and Left_IC_Val2)
					 * Left_Distance = total Left_Distance/2 (as the wave goes then returns back)
					 * Left_Distance = 17000*echo time
					 *
					 * as the cpu works with 168MHz freq and prescaler of 168
					 * then to execute 1 second we need 1us
					 *
					 *therefore, distace = 17000*echo time* 1us = 0.017*echotime
					 *
					 */

					Left_Distance = Left_difference * 0.0175;
					Left_First_Captured_Flag = 0; // set it back to false

					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
					// xQueueSendFromISR(xDistanceQueue, &Left_Distance, NULL);
				}
			}
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
			{
				// To check if the first value is not captured yet
				if (Rear_First_Captured_Flag==0)
				{
					// stores the first value
					Rear_IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
					// set the first captured as true as we now needs to capture the falling edge
					Rear_First_Captured_Flag = 1;
					// Now change the polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				// to check if the first is already captured
				else if (Rear_First_Captured_Flag==1)
				{
					// stores second value
					Rear_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
					// reset the counter
					__HAL_TIM_SET_COUNTER(htim, 0);

					if (Rear_IC_Val2 > Rear_IC_Val1)
					{
						Rear_difference = Rear_IC_Val2-Rear_IC_Val1;
					}

					else if (Rear_IC_Val1 > Rear_IC_Val2)
					{
						// to check for any overflows
						Rear_difference = (0xffff - Rear_IC_Val1) + Rear_IC_Val2;
					}

					/*Then to Calculate the Rear_Distance:
					 *
					 * speed of sound in air = 344m/s = 34400 cm/s
					 * total Rear_Distance = speed * echo time (diff bet Rear_IC_Val1 and Rear_IC_Val2)
					 * Rear_Distance = total Rear_Distance/2 (as the wave goes then returns back)
					 * Rear_Distance = 17000*echo time
					 *
					 * as the cpu works with 168MHz freq and prescaler of 168
					 * then to execute 1 second we need 1us
					 *
					 *therefore, distace = 17000*echo time* 1us = 0.017*echotime
					 *
					 */

					Rear_Distance = Rear_difference * 0.0174;
					Rear_First_Captured_Flag = 0; // set it back to false

					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
					// xQueueSendFromISR(xDistanceQueue, &Rear_Distance, NULL);
				}
			}

}

uint32_t Distance_Covered(void)
{
	return Right_Distance;
}
