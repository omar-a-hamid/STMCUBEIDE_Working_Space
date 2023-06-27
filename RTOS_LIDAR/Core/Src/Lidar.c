/******************************************************************
 *  lidar.c
 *
 *  Created on: Apr 20, 2022
 *
 *  Author:Nour Eldin
 *******************************************************************/
#include"Lidar.h"

uint16_t Global_u16LidarDistance=500;
uint16_t strength;
uint16_t temp;

uint8_t Lidar_get_distance(uint8_t *buf, uint32_t len) {

	uint32_t i = 0;

	uint8_t error_state = ERROR_FRMAE;

	uint8_t chk_cal = 0;

	/*check frame length  */
	if (TFMINI_DATA_Len == len) {
		if ((TFMINT_DATA_HEAD == buf[0]) && (TFMINT_DATA_HEAD == buf[1])) {
			for (i = 0; i < (TFMINI_DATA_Len - 1); i++) {
				chk_cal += buf[i];
			}
			/*check if sum of byte 0 to byte 7 =check sum*/
			if (chk_cal == buf[TFMINI_DATA_Len - 1]) {
				strength= buf[4] | (buf[5] << 8);
                temp=buf[6] | (buf[7] << 8);
				Global_u16LidarDistance = buf[2] | (buf[3] << 8);

				error_state = CORECT_FRAME;

			} else {

				/*Error in frame */
			}
		}
	}
	return error_state;
}

