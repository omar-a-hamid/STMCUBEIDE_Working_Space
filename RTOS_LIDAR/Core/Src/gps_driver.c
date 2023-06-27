/*
 * gps_driver.c
 *
 *  Created on: Dec 10, 2022
 *      Author: Nourhan Hussein
 */

#include "gps_driver.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"



int GPS_validate(char *GGA_buffer)
{
	int i=0;

	if(GGA_buffer[i] == '$')
	{
		i++;

		return 0;
	}
	else
	{
		return 1; //ERROR
	}
}


void GGA_decode(char *GGA_buffer, GGA_STRUCT *GGA)
{
	/***********separate each data to check the quality indicator************/
	int idx=0;
	int i=0;
	int j = 0;

	/*
	 * difference in time between cairo and GMT is 2 hours, 432000 seconds
	 */
	int GMT=300;

	int hr=0;
	int min=0;
	int sec=0;

	int day_change = 0;
	/*
	 * variables for position
	 */
	int decimal_length;
	int decimal;

	float latitude;
	float longitude;
	float altitude;
	/*
	int16_t lat;
	int16_t longt;
	int16_t alt;
	 */
	int16_t temp;
	/*
	 * buffer to store data from NMEA message
	 * max number of characters in each parameter is 12
	 */
	char data_buffer[12];

	idx=0;
	while (GGA_buffer[idx] != ',')
	{
		idx++;  // 1st ',' // starting from index 0
	}

	////*************GET TIME*****************/////

	idx++;
	// reach the first number in time

	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.
	i=0; //resetting i

	while(GGA_buffer[idx] != ',') //copying the time data in a buffer
	{
		data_buffer[i]= GGA_buffer[idx];
		i++;
		idx++;
	}

	/*
	 * atoi function converts an integer value from a string of characters.
	 */

	hr = (atoi(data_buffer)/10000) + GMT/100;   // get the hours from the 6 digit number

	min = ((atoi(data_buffer)/100)%100) + GMT%100;  // get the minutes from the 6 digit number

	sec = (atoi(data_buffer)%100);

	if (sec > 59)
	{
		sec -= 60;
		min++;
	}
	if (min > 59)
	{
		min -= 60;
		hr++;
	}
	if (hr<0)
	{
		hr+=24;
		day_change--;
	}
	if (hr>=24)
	{
		hr-=24;;
		day_change++;
	}

	/*
	 * passing the time data values to GGA structure
	 */

	GGA->time.hour = hr;
	GGA->time.minuit = min;
	GGA->time.second = sec;

	////*************GET LATITUDE*****************/////

	idx++;
	// reach the first number in latitude

	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.
	i=0; //resetting i

	while(GGA_buffer[idx] != ',') //copying the time data in a buffer
	{
		data_buffer[i]= GGA_buffer[idx];
		i++;
		idx++;
	}
	/*
	 * error for str length
	 */
	temp = (atoi(data_buffer));   // change the buffer to the number. It will only convert to integers

	j = 0;

	/*
	 * counting the number of digits before the decimal
	 */
	while (data_buffer[j] != '.')
	{
		j++;
	}
	j++;

	decimal_length = (strlen(data_buffer))-j;  // calculate the number of digit after decimal

	decimal = atoi ((char *) data_buffer+j);  // separate the decimal numbers from the data_buffer

	/*
	 * getting the number before the decimal (temp/100.0) with its decimal points by using explicit casting (100.0)
	 * adding the decimal numbers to it and dividing it with a high power to shift them
	 * ex:
	 * latitude from NMEA is 1234.5678
	 * temp =1234 -> 12.34
	 * decimal= 5678 -> 0.005678
	 * latitude = 12.345678
	 */
	latitude = (temp/100)+(((temp-((temp/100)*100))+(decimal/pow(10, (decimal_length))))/60);

	GGA ->position.latitude = latitude;

	////*************GET LATITUDE DIRECTION*****************/////
	idx++; //north or south
	GGA ->position.NORTH_SOUTH = data_buffer[idx];
	idx++; // ',' after north or south

	////*************GET LONGTIUDE*****************/////

	idx++;
	// reach the first number in latitude

	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.
	i=0; //resetting i

	while(GGA_buffer[idx] != ',') //copying the time data in a buffer
	{
		data_buffer[i]= GGA_buffer[idx];
		i++;
		idx++;
	}
	/*
	 * error for str length
	 */
	temp = (atoi(data_buffer));   // change the buffer to the number. It will only convert to integers

	j = 0;

	/*
	 * counting the number of digits before the decimal
	 */
	while (data_buffer[j] != '.')
	{
		j++;
	}
	j++;

	decimal_length = (strlen(data_buffer))-j;  // calculate the number of digit after decimal

	decimal = atoi ((char *) data_buffer+j);  // separate the decimal numbers from the data_buffer

	/*
	 * getting the number before the decimal (temp/100.0) with its decimal points by using explicit casting (100.0)
	 * adding the decimal numbers to it and dividing it with a high power to shift them
	 * ex:
	 * longitude from NMEA is 1234.5678
	 * temp =1234 -> 12.34
	 * decimal= 5678 -> 0.005678
	 * longitude = 12.345678
	 */
	longitude = (temp/100)+(((temp-((temp/100)*100))+(decimal/pow(10, (decimal_length))))/60);

	GGA ->position.logitude = longitude;

	////*************GET LATITUDE DIRECTION*****************/////
	idx++; //east or west
	GGA ->position.EAST_WEST = data_buffer[idx];
	idx++; // ',' after east or west

	////*************POSITION FIX*****************/////

	idx++;  // reached the character to identify the fix

	if ((GGA_buffer[idx] == '1') || (GGA_buffer[idx] == '2') || (GGA_buffer[idx] == '6'))   // 0 indicates no fix yet
	{
		GGA->quality_indicator = 1;   // fix available
		idx++; // ',' after position fix
	}
	else
	{
		GGA->quality_indicator = 0;   // If the fix is not available
	}

	////*************NUMBER OF SATTELITES*****************/////
	idx++; // number of sattelites

	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.
	i=0; //resetting i

	while(GGA_buffer[idx] != ',') //copying the time data in a buffer
	{
		data_buffer[i]= GGA_buffer[idx];
		i++;
		idx++;
	}
	GGA ->number_of_sattelites= atoi(data_buffer);
	idx++; // ',' after number of sat

	////*************GET ALTITUDE*****************/////

	idx++;
	// reach the first number in altitude

	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.
	i=0; //resetting i

	while(GGA_buffer[idx] != ',') //copying the time data in a buffer
	{
		data_buffer[i]= GGA_buffer[idx];
		i++;
		idx++;
	}
	/*
	 * error for str length
	 */
	temp = (atoi(data_buffer));   // change the buffer to the number. It will only convert to integers

	j = 0;

	/*
	 * counting the number of digits before the decimal
	 */
	while (data_buffer[j] != '.')
	{
		j++;
	}
	j++;

	decimal_length = (strlen(data_buffer))-j;  // calculate the number of digit after decimal

	decimal = atoi ((char *) data_buffer+j);  // separate the decimal numbers from the data_buffer

	/*
	 * getting the number before the decimal (temp/100.0) with its decimal points by using explicit casting (100.0)
	 * adding the decimal numbers to it and dividing it with a high power to shift them
	 * ex:
	 * altitude from NMEA is 1234.5678
	 * temp =1234 -> 12.34
	 * decimal= 5678 -> 0.005678
	 * altitude = 12.345678
	 */
	altitude = (temp/100)+(((temp-((temp/100)*100))+(decimal/pow(10, (decimal_length))))/60);

	GGA ->altitude.altitude = altitude;

	////*************GET ALTITUDE DIRECTION*****************/////
	idx++; //altitude unit
	GGA ->altitude.meters = data_buffer[idx];




}

void RMC_decode(char *RMC_buffer, RMC_STRUCT *RMC)
{

	int decimal_length;
	int decimal;

	float speed;
	float course;


	int day=0;
	int month=0;
	int year=0;

	int16_t temp;

	char data_buffer[12];
	int idx=0;
	int i = 0;
	int j = 0;

	idx = 0;
	while (RMC_buffer[idx] != ',')
	{
		idx++;  // 1st ','
	}

	idx++; //time


	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.
	i=0; //resetting i

	while (RMC_buffer[idx] != ',')
	{
		idx++;  // ',' After time
	}

	idx++; //validity

	/*
	 *  'A' Indicates the data is valid
	 *  'V' indicates invalid data
	 */
	if (RMC_buffer[idx] == 'A')
	{
		RMC->validity_status= 1;
	}
	else
	{
		RMC->validity_status =0;
	}

	idx++; // ',' after validity
	/*
	 * skipping these parameters as we got them from GGA
	 */

	idx++; //latitude
	while (RMC_buffer[idx] != ',')
	{
		idx++;  // ',' after latitude
	}
	idx++; //north or south
	while (RMC_buffer[idx] != ',')
	{
		idx++;  // ',' after north or south
	}
	idx++; //longitude
	while (RMC_buffer[idx] != ',')
	{
		idx++;  // ',' after longitude
	}
	idx++; //east or west
	while (RMC_buffer[idx] != ',')
	{
		idx++;  // ',' after east or west
	}

	/************************************************GET SPEED*****************************************************/
	idx++; // speed
	memset(data_buffer, '\0', 12);
	i=0;

	while (RMC_buffer[idx] != ',')
	{
		data_buffer[i] = RMC_buffer[idx];
		i++;
		idx++;
	}

	//// if the speed have some data
	if (strlen (data_buffer) > 0)
	{
		temp = (atoi(data_buffer));  // convert the data into the number
		j = 0;
		while (data_buffer[j] != '.')
		{
			j++;   // same as above
		}
		j++;

		decimal_length = (strlen(data_buffer))-j;
		decimal = atoi ((char *) data_buffer+j);

		speed = temp + (decimal/pow(10, (decimal_length)));

		RMC->speed_over_gnd = floor(speed*2.1);
	}

	else RMC->speed_over_gnd = 0;

	/************************************************GET COURSE*****************************************************/
	idx++; // ','speed
//	memset(data_buffer, '\0', 12);
//	i=0;

	while (RMC_buffer[idx] != ',')
	{
//		data_buffer[i] = RMC_buffer[idx];
//		i++;
		idx++;
	}

	// if the speed have some data
	if (strlen (data_buffer) > 0)
	{
		temp = (atoi(data_buffer));  // convert the data into the number
		j = 0;
		while (data_buffer[j] != '.')
		{
			j++;   // same as above
		}
		j++;

		decimal_length = (strlen(data_buffer))-j;
		decimal = atoi ((char *) data_buffer+j);

		course = temp + (decimal/pow(10, (decimal_length)));

		RMC->course_over_gnd = course;
	}

	else RMC->course_over_gnd = 0;

	////****************************************GET DATE************************************************/////

	idx++;

	// reach the first number in time
	i=0; //resetting i

	memset(data_buffer, '\0', 12); //memset is a function to fill a block of memory with a particular value.

	while(RMC_buffer[idx] != ',') //copying the time data in a buffer
	{
		data_buffer[i]= RMC_buffer[idx];
		i++;
		idx++;
	}

	/*
	 * ex.
	 * date:131222
	 */

	day = atoi(data_buffer)/10000;   // get 13

	month = ((atoi(data_buffer)/100)%100);  // get 12

	year = (atoi(data_buffer)%100); //get 22

	RMC->date.day=day;
	RMC->date.month=month;
	RMC->date.year=year;

}
