/******************************************
 * Lidar.h
 *
 *  Created on: Apr 20, 2022
 *
 *  Author: Nour Eldin
 */

#ifndef SRC_LIDAR_H_
#define SRC_LIDAR_H_

 #include <stdint.h>

/*define head of the LIDAR_FRAME */
#define TFMINT_DATA_HEAD            0x59
#define USART_BUF_SIZE              64  /*USART_BUFFER_SIZE*/
#define ERROR_FRMAE                 0

#define CORECT_FRAME                1

/*define Frame Length of LIDAR */
#define TFMINI_DATA_Len             9

/*global variable */


/*define function prototype
 * input:frame that received from uart
 *       frame length
 * output:error state                 */

uint8_t Lidar_get_distance(uint8_t *buf, uint32_t len);


#endif /* SRC_LIDAR_H_ */
