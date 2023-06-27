/*
 * gps_driver.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Nourhan Hussein
 */

#ifndef INC_GPS_DRIVER_H_
#define INC_GPS_DRIVER_H_

 //*******************************************Definitions************************************************//

#define GPS_BUFFER_SIZE  128

 //*******************************************Structures definition**************************************//
/*
 * struct for time to get hr, min, sec
 */
typedef struct
{
	int hour;
	int minuit;
	int second;
} TIME;

/*
 * structure for location to get the longitude, latitude, N/S , E/W
 */
typedef struct
{
	float logitude;
	float latitude;
	char NORTH_SOUTH;
	char EAST_WEST;
}POSITION;

/*
 * altitude is the height above sea level, it is not with the position structure, as it is used in GGA not RMC
 */
typedef struct
{
	double altitude;
	char meters;
}ALTITUDE;

typedef struct
{
	int day;
	int month;
	int year;
}DATE;

/*
 *structure for GGA -> Global Positiong System fix data
 *differs from RMC in:
 * 1))Quality indicator: 0 -> Fix not available
 * 1 -> Fix valid
 * 2 -> Fix valid, Differential GPS
 * 6 -> Dead reckoning
 *
 * 2))indicating number of satellites used
 */
typedef struct
{
	TIME time;
	POSITION position;
	ALTITUDE altitude;
	int quality_indicator;
	int number_of_sattelites;
} GGA_STRUCT;

/*
 *structure for RMC -> Recommended minimum specific GNSS data
 *differs from GGA in:
 * 1))STATUS: 0 -> NOT VALID
 * 1 -> VALID
 *
 * 2))speed over ground
 * 3))course over ground
 */
typedef struct
{
	DATE date;
	int validity_status;
	float speed_over_gnd; //knot
	int course_over_gnd; //degree

} RMC_STRUCT;

/*
 * to get the position RMC STRUCT and GGA STRUCT will be combined together in GPS STRUCT
 */

typedef struct
{
	GGA_STRUCT GGA;
	RMC_STRUCT RMC;
}GPS_STRUCT;

//*******************************************Function definition**************************************//
int GPS_validate(char *GGA_buffer);


/*
 * GGA function to decode the data from GGA
 */
void GGA_decode(char *GGA_buffer, GGA_STRUCT *GGA);

/*
 * RMC function to decode the data from RMC
 */

void RMC_decode(char *RMC_buffer, RMC_STRUCT *RMC);

#endif /* INC_GPS_DRIVER_H_ */
