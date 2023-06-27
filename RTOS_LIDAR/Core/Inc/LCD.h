
 /******************************************************************************
 *
 * Module: LCD
 *
 * File Name: LCD.h
 *
 * Description: Header file for the LCD driver for STM
 *
 * Author: Omar Shamekh
 *
 *******************************************************************************/

#ifndef LCD_H_
#define LCD_H_



/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* LCD Data bits mode configuration, its value should be 4 or 8*/
#define LCD_DATA_BITS_MODE 4

#if((LCD_DATA_BITS_MODE != 4) && (LCD_DATA_BITS_MODE != 8))

#error "Number of Data bits should be equal to 4 or 8"

#endif

#if (LCD_DATA_BITS_MODE == 4)

/* if LCD_LAST_PORT_PINS is defined in the code, the LCD driver will use the last 4 pins in the gpio port for for data.
 * To use the first four pins in the gpio port for data just remove LCD_LAST_PORT_PINS */
//#define LCD_LAST_PORT_PINS

#ifdef LCD_LAST_PORT_PINS
#define LCD_FIRST_DATA_PIN        999
#else
#define LCD_FIRST_DATA_PIN         222
#endif

#endif

/* LCD HW Ports and Pins Ids */
#define LCD_RS_PORT_ID                GPIOD
#define LCD_RS_PIN_ID                  GPIO_PIN_0

#define LCD_RW_PORT_ID                 GPIOD
#define LCD_RW_PIN_ID                  GPIO_PIN_1

#define LCD_E_PORT_ID                  GPIOD
#define LCD_E_PIN_ID                   GPIO_PIN_2

#define DATA_PORT_ID GPIOB

#define D0_PIN GPIO_PIN_0
#define D1_PIN GPIO_PIN_1
#define D2_PIN GPIO_PIN_2
#define D3_PIN GPIO_PIN_3
#define D4_PIN GPIO_PIN_4
#define D5_PIN GPIO_PIN_5
#define D6_PIN GPIO_PIN_6
#define D7_PIN GPIO_PIN_7
#include <stdint.h>

/* LCD Commands */
#define LCD_CLEAR_COMMAND              0x01
#define LCD_GO_TO_HOME                 0x02
#define LCD_TWO_LINES_EIGHT_BITS_MODE  0x38
#define LCD_TWO_LINES_FOUR_BITS_MODE   0x28
#define LCD_CURSOR_OFF                 0x0C
#define LCD_CURSOR_ON                  0x0E
#define LCD_SET_CURSOR_LOCATION        0x80

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*
 * Description :
 * Initialize the LCD:
 * 1. Setup the LCD pins directions by use the GPIO driver.
 * 2. Setup the LCD Data Mode 4-bits or 8-bits.
 */
void LCD_init(void);

/*
 * Description :
 * Send the required command to the screen
 */
void LCD_sendCommand(unsigned char  command);

/*
 * Description :
 * Display the required character on the screen
 */
void LCD_displayCharacter(unsigned char  data);

/*
 * Description :
 * Display the required string on the screen
 */
void LCD_displayString(const char *Str);

/*
 * Description :
 * Move the cursor to a specified row and column index on the screen
 */
void LCD_moveCursor(unsigned char  row,unsigned char  col);

/*
 * Description :
 * Display the required string in a specified row and column index on the screen
 */
void LCD_displayStringRowColumn(unsigned char  row,unsigned char  col,const char *Str);

/*
 * Description :
 * Display the required decimal value on the screen
 */
void LCD_intgerToString(uint32_t data);

/*
 * Description :
 * Send the clear screen command
 */
void LCD_clearScreen(void);

void writePort_8bit(unsigned char datain);
void writePort_4bit(unsigned char datain);


#endif /* LCD_H_ */

