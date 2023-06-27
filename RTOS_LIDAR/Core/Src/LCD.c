/******************************************************************************
 *
 * Module: LCD
 *
 * File Name: LCD.c
 *
 * Description: Source file for the LCD for STM
 *
 * Author: Omar Shamekh
 *
 *******************************************************************************/
#include "LCD.h"

#include "stm32f4xx_hal.h"

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

/*
 * Description :
 * Initialize the LCD:
 * 1. Setup the LCD pins directions by use the GPIO driver.
 * 2. Setup the LCD Data Mode 4-bits or 8-bits.
 */
void LCD_init(void)
{
	/* Configure the direction for RS, RW and E pins as output pins */


#if (LCD_DATA_BITS_MODE == 4)


	LCD_sendCommand(LCD_GO_TO_HOME);
	LCD_sendCommand(LCD_TWO_LINES_FOUR_BITS_MODE); /* use 2-line lcd + 4-bit Data Mode + 5*7 dot display Mode */

#elif (LCD_DATA_BITS_MODE == 8)
	/* Configure the data port as output port */
	LCD_sendCommand(LCD_TWO_LINES_EIGHT_BITS_MODE); /* use 2-line lcd + 8-bit Data Mode + 5*7 dot display Mode */
#endif

	LCD_sendCommand(LCD_CURSOR_OFF); /* cursor off */
	LCD_sendCommand(LCD_CLEAR_COMMAND); /* clear LCD at the beginning */
}

/*
 * Description :
 * Send the required command to the screen
 */
void LCD_sendCommand(unsigned char  command)
{
#if (LCD_DATA_BITS_MODE == 8)
	writePort_8bit(command);
	HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RW_PORT_ID,LCD_RW_PIN_ID,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,GPIO_PIN_RESET);

	/* Data Mode RS=1 */
	 /* write data to LCD so RW=0 */
	HAL_Delay(10); /* delay for processing Tas = 50ns */
	 /* Enable LCD E=1 */
	HAL_Delay(10); /* delay for processing Tpw - Tdws = 190ns */
	HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_RESET);
#elif (LCD_DATA_BITS_MODE == 4)
	unsigned char val = 0;
#if (LCD_FIRST_DATA_PIN  == 999)
	val= command & 0xF0;
#elif (LCD_FIRST_DATA_PIN  == 222)
	val= (command>>4) & 0x0F;
#endif

		writePort_4bit(val);
		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_RW_PORT_ID,LCD_RW_PIN_ID,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,GPIO_PIN_RESET);

		/* Data Mode RS=1 */
		 /* write data to LCD so RW=0 */
		HAL_Delay(2); /* delay for processing Tas = 50ns */

		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_RESET);

#if (LCD_FIRST_DATA_PIN  == 999)
		val= ((command<<4) & 0xF0);
#elif (LCD_FIRST_DATA_PIN  == 222)
		val= (command & 0x0F);
#endif
		writePort_4bit(val);
		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_RW_PORT_ID,LCD_RW_PIN_ID,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,GPIO_PIN_RESET);

		/* Data Mode RS=1 */
		 /* write data to LCD so RW=0 */
		HAL_Delay(2); /* delay for processing Tas = 50ns */

		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_RESET);

#endif

}

/*
 * Description :
 * Display the required character on the screen
 */
void LCD_displayCharacter(unsigned char  data)
{
#if (LCD_DATA_BITS_MODE == 8)
	writePort_8bit(data);
	HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RW_PORT_ID,LCD_RW_PIN_ID,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,GPIO_PIN_SET);

	/* Data Mode RS=1 */
	 /* write data to LCD so RW=0 */
	HAL_Delay(10); /* delay for processing Tas = 50ns */
	 /* Enable LCD E=1 */
	HAL_Delay(10); /* delay for processing Tpw - Tdws = 190ns */
	HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_RESET);
#elif (LCD_DATA_BITS_MODE == 4)
	unsigned char val = 0;
#if (LCD_FIRST_DATA_PIN  == 999)
	val= data & 0xF0;
#elif (LCD_FIRST_DATA_PIN  == 222)
	val= (data>>4) & 0x0F;
#endif

		writePort_4bit(val);
		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_RW_PORT_ID,LCD_RW_PIN_ID,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,GPIO_PIN_SET);

		/* Data Mode RS=1 */
		 /* write data to LCD so RW=0 */
		HAL_Delay(2);
		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_RESET);
#if (LCD_FIRST_DATA_PIN  == 999)
		val= ((data<<4) & 0xF0);
#elif (LCD_FIRST_DATA_PIN  == 222)
		val= (data & 0x0F);
#endif

		writePort_4bit(val);
		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_RW_PORT_ID,LCD_RW_PIN_ID,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,GPIO_PIN_SET);

		/* Data Mode RS=1 */
		 /* write data to LCD so RW=0 */
		HAL_Delay(2); /* delay for processing Tas = 50ns */

		HAL_GPIO_WritePin(LCD_E_PORT_ID,LCD_E_PIN_ID,GPIO_PIN_RESET);

#endif
}



/*
 * Description :
 * Display the required string on the screen
 */
void LCD_displayString(const char *Str)
{
	unsigned char  i = 0;
//	while(Str[i] != '\0')
//	{
//		LCD_displayCharacter(Str[i]);
//		i++;
//	}
	/***************** Another Method ***********************/
	while((*Str) != '\0')
	{
		LCD_displayCharacter(*Str);
		Str++;
	}
	 /*********************************************************/
}

/*
 * Description :
 * Move the cursor to a specified row and column index on the screen
 */
void LCD_moveCursor(unsigned char  row,unsigned char  col)
{
	unsigned char  lcd_memory_address;

	/* Calculate the required address in the LCD DDRAM */
	switch(row)
	{
	case 0:
		lcd_memory_address=col;
		break;
	case 1:
		lcd_memory_address=col+0x40;
		break;
	case 2:
		lcd_memory_address=col+0x10;
		break;
	case 3:
		lcd_memory_address=col+0x50;
		break;
	}
	/* Move the LCD cursor to this specific address */
	LCD_sendCommand(lcd_memory_address | LCD_SET_CURSOR_LOCATION);
}

/*
 * Description :
 * Display the required string in a specified row and column index on the screen
 */
void LCD_displayStringRowColumn(unsigned char  row,unsigned char  col,const char *Str)
{
	LCD_moveCursor(row,col); /* go to to the required LCD position */
	LCD_displayString(Str); /* display the string */
}

/*
 * Description :
 * Display the required decimal value on the screen
 */
void LCD_intgerToString(uint32_t data)
{
	char buff[16]; /* String to hold the ascii result */
	itoa(data,buff,10); /* Use itoa C function to convert the data to its corresponding ASCII value, 10 for decimal */
	LCD_displayString(buff); /* Display the string */
}

/*
 * Description :
 * Send the clear screen command
 */
void LCD_clearScreen(void)
{
	LCD_sendCommand(LCD_CLEAR_COMMAND); /* Send clear display command */
}
void writePort_8bit(unsigned char datain)
{

	HAL_GPIO_WritePin(DATA_PORT_ID, D7_PIN, ((datain>>7)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D6_PIN, ((datain>>6)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D5_PIN, ((datain>>5)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D4_PIN, ((datain>>4)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D3_PIN, ((datain>>3)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D2_PIN, ((datain>>2)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D1_PIN, ((datain>>1)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D0_PIN, ((datain>>0)&0x01));

}
void writePort_4bit(unsigned char datain)
{
#if (LCD_FIRST_DATA_PIN  == 999)
	HAL_GPIO_WritePin(DATA_PORT_ID, D7_PIN, ((datain>>7)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D6_PIN, ((datain>>6)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D5_PIN, ((datain>>5)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D4_PIN, ((datain>>4)&0x01));
#elif (LCD_FIRST_DATA_PIN  == 222)
	HAL_GPIO_WritePin(DATA_PORT_ID, D3_PIN, ((datain>>3)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D2_PIN, ((datain>>2)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D1_PIN, ((datain>>1)&0x01));
	HAL_GPIO_WritePin(DATA_PORT_ID, D0_PIN, ((datain>>0)&0x01));
#endif
}
