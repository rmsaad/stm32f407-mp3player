/*
 * lcm1602a_driver.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Rami
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"

/*Speaker*/
uint8_t Speaker[8] = {
	0b00001,
	0b00011,
	0b00111,
	0b11111,
	0b11111,
	0b00111,
	0b00011,
	0b00001
};

/*Volume 1*/
uint8_t Volume_1[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b11000,
	0b11000
};

/*Volume 2*/
uint8_t Volume_2[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00011,
	0b00011,
	0b11011,
	0b11011
};

/*Volume 3*/
uint8_t Volume_3[8] = {
	0b00000,
	0b00000,
	0b11000,
	0b11000,
	0b11000,
	0b11000,
	0b11000,
	0b11000
};

/*Volume 4*/
uint8_t Volume_4[8] = {
	0b00011,
	0b00011,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b11011
};

/* Private static variables ---------------------------------------------------------*/

static GPIO_TypeDef *data_port;
static GPIO_TypeDef *control_port;

static uint16_t data_pins[8];
static uint16_t control_pins[3];

static uint16_t data_7_pin;
static uint8_t transmission_mode;

/* Private static function prototypes -----------------------------------------------*/

static void create_char(uint8_t location, uint8_t charmap[]);
static void LCM1602a_Write8_Message(char *Message);
static void LCM1602a_Write4_Message(char *Message);
static void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
static void LCM1602a_Write4_Data_Single(uint8_t dataValues, uint8_t RS, uint8_t RW);
static void LCM1602a_Write4_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
static void LCM1602a_Hang_Busy_Flag();


/**
  * @brief  Sets data and control pins for use in 8 Data-Line transmission
  * @param  Data port
  * @param  Data pins
  * @param  Control port
  * @param  Control pins
  * @retval None
  */
void LCM1602a_Set_DATA8(GPIO_TypeDef *d_Port, uint16_t d_Pins[8], GPIO_TypeDef *c_Port, uint16_t c_Pins[3]){

	transmission_mode = DATA_8;															/*set transmission mode*/
	data_port = d_Port;																	/*set data port*/
	control_port = c_Port;																/*set control port*/
	data_7_pin = d_Pins[7];																/*set D7 pin*/

	for(int i = 0; i < DATA_8; i++){													/*set data pins*/
		data_pins[i] = d_Pins[i];														/* " " " */
	}

	for(int i = 0; i < CONTROL_PIN_COUNT; i++){											/*set control pins*/
		control_pins[i] = c_Pins[i];													/* " " " */
	}

}

/**
  * @brief  Sets data and control pins for use in 4 Data-Line transmission
  * @param  Data port
  * @param  Data pins
  * @param  Control port
  * @param  Control pins
  * @retval None
  */
void LCM1602a_Set_DATA4(GPIO_TypeDef *d_Port, uint16_t d_Pins[4], GPIO_TypeDef *c_Port, uint16_t c_Pins[3]){

	transmission_mode = DATA_4;															/*set transmission mode*/
	data_port = d_Port;																	/*set data port*/
	control_port = c_Port;																/*set control port*/
	data_7_pin = d_Pins[3];																/*set D7 pin*/

	for(int i = 0; i < DATA_4; i++){													/*set data pins*/
		data_pins[i] = d_Pins[i];														/* " " " */
	}

	for(int i = 0; i < CONTROL_PIN_COUNT; i++){											/*set control pins*/
		control_pins[i] = c_Pins[i];													/* " " " */
	}

}

/**
  * @brief  store custom 5x8 character into LCD RAM
  * @param  location in ram
  * @param  character as uint8_t map
  * @retval None
  */
static void create_char(uint8_t location, uint8_t charmap[]){
	location &= 0x7; 																	/*we only have 8 locations 0-7*/
	LCM1602a_Write_Data(0x40 | (location << 3), 0, 0);									/**/

	for(int i = 0; i < 8; i++){															/*write char into RAM*/
		LCM1602a_Write_Data(charmap[i], 1, 0);											/* " " " */
	}
}

/**
  * @brief  initialization process for LCM1602a
  * @param  how many LCD dispay Lines
  * @retval None
  */
//TODO test for DATA4 Transmission Later
void LCM1602a_init(uint8_t disp_line){

	if(transmission_mode == DATA_8){
		create_char(0, Speaker);														/*store custom characters into LCD RAM*/
		create_char(1, Volume_1);														/* " " " */
		create_char(2, Volume_2);														/* " " " */
		create_char(3, Volume_3);														/* " " " */
		create_char(4, Volume_4);														/* " " " */

		if(disp_line == TWO_LINE_DISPLAY){
			LCM1602a_Write_Data(0b00111000, 0, 0);										/*Initialize the display mode 2 Line*/
		}else if(disp_line == ONE_LINE_DISPLAY){
			LCM1602a_Write_Data(0b00110000, 0, 0);										/*Initialize the display mode 1 Line*/
		}

		LCM1602a_Write_Data(0b00001100, 0, 0);											/*Display on With Cursor off*/
		LCM1602a_Write_Data(0b00000110, 0, 0);											/*increment on*/
		LCM1602a_Write_Data(0b00000001, 0, 0);											/*clear the display*/

	}else if(transmission_mode == DATA_4){

		LCM1602a_Write4_Data_Single(0b0011, 0, 0); HAL_Delay(5);
		LCM1602a_Write4_Data_Single(0b0011, 0, 0); HAL_Delay(5);
		LCM1602a_Write4_Data_Single(0b0011, 0, 0); HAL_Delay(5);

		LCM1602a_Write4_Data_Single(0b0010, 0, 0); HAL_Delay(5);
		LCM1602a_Write_Data(0b00101000, 0, 0);											/*Initialize the display mode 1 Line*/

		LCM1602a_Write_Data(0b00001100, 0, 0);											/*Display on With Cursor off*/
		LCM1602a_Write_Data(0b00000110, 0, 0);											/*increment on*/
		LCM1602a_Write_Data(0b00000001, 0, 0);											/*clear the display*/


	}

}

/**
  * @brief  wraps text arround LCD display
  * @param  char pointer to the ascii text to be displayed on the LCD screen
  * @retval None
  */
void LCM1602a_textwrap(char* in_text, uint8_t delay){

	char text[70];
	sprintf(text, "%s%s%s", "        ", in_text ,"        ");

	static uint8_t delay_factor = 0;
	static uint16_t cursor_pos = 0;

	if (delay_factor != delay){
		delay_factor++;
		return;
	}

	int text_len = strlen(text);														/*find length of text*/

	if (cursor_pos == (text_len - 1) ){                                             	/*reset cursor for wrapping*/
		cursor_pos = 0;																	/* " " " */
	}


	LCM1602a_Write_Data(0b00000010, 0, 0);												/*return display home*/
	//LCM1602a_Write_Data(0b11000000, 0, 0);											/*second line IMPLEMENT LATER*/

	if(cursor_pos < text_len - 16){                                                 	/*first 16 characters*/
		for (int char_pos = cursor_pos; char_pos < cursor_pos + 16 ; char_pos++){		/* " " " */
			LCM1602a_Write_Data((int)text[char_pos], 1, 0);								/* " " " */
		}
	}

	else{
		for (int char_pos = cursor_pos; char_pos < (text_len - 1) ; char_pos++){        /*characters of current string*/
			LCM1602a_Write_Data((int)text[char_pos], 1, 0);								/* " " " */
		}

		for (int char_pos = 0; char_pos <= 16 - (text_len - cursor_pos); char_pos++){   /*remaining characters*/
			LCM1602a_Write_Data((int)text[char_pos], 1, 0);								/* " " " */
		}
	}

	cursor_pos++;
	delay_factor = 0;
}

/**
  * @brief  Writes a char array to the LCD screen
  * @param  char pointer to the ascii message to be displayed on the LCD screen
  * @retval None
  */
void LCM1602a_Write_Message(char *Message){
	if (transmission_mode == DATA_8){													/*if 8 Data Lines*/
		LCM1602a_Write8_Message(Message);													/*use corresponding write function*/
	}else if(transmission_mode == DATA_4){												/*if 4 Data Lines*/
		LCM1602a_Write4_Message(Message);													/*use corresponding write function*/
	}
}

/**
  * @brief  Writes a char array to the LCD screen (8 Data-Lines)
  * @param  char pointer to the ascii message to be displayed on the LCD screen
  * @retval None
  */
static void LCM1602a_Write8_Message(char *Message){

	uint16_t Len = (uint16_t)strlen(Message);											/*find length of the message*/

	for(int i = 0; i < Len; i++){														/*write message to display*/
		LCM1602a_Write8_Data((int)*Message, 1, 0);										/* " " " */
		Message++;																		/* " " " */
	}
}

/**
  * @brief  Writes a char array to the LCD screen (4 Data-Lines)
  * @param  char pointer to the ascii message to be displayed on the LCD screen
  * @retval None
  */
static void LCM1602a_Write4_Message(char *Message){

	uint16_t Len = (uint16_t)strlen(Message);											/*find length of the message*/

	for(int i = 0; i < Len; i++){														/*write message to display*/
		LCM1602a_Write4_Data((int)*Message, 1, 0);										/* " " " */
		Message++;																		/* " " " */
	}
}

void LCM1602a_Write_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){
	if (transmission_mode == DATA_8){													/*if 8 Data Lines*/
		LCM1602a_Write8_Data(dataValues, RS, RW);											/*use corresponding write function*/
	}else if(transmission_mode == DATA_4){												/*if 4 Data Lines*/
		LCM1602a_Write4_Data(dataValues, RS, RW);											/*use corresponding write function*/
	}
}

/**
  * @brief  sets data pins, followed by RS and RW control pins with instruction code being sent to the LCD when the E pin is toggled
  * @param  8 bit data value
  * @param  RS value
  * @param  RW value
  * @retval None
  */
static void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	LCM1602a_Hang_Busy_Flag();															/*hang until busy flag is reset*/

	for(int i = 0; i < DATA_8; i++){													/*write to data lines*/
		HAL_GPIO_WritePin(data_port, data_pins[i] , ((dataValues >> i) & 1));			/* " " " */
	}																					/* " " " */


	HAL_GPIO_WritePin(control_port, control_pins[0], RS);								/*write to control lines RS, RW*/
	HAL_GPIO_WritePin(control_port, control_pins[1], RW);								/* " " " */

	HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);						/*set E to High*/

	for(int i = CONTROL_PIN_COUNT - 1; i >= 0; i--){ 									/*reset all control pins*/
		HAL_GPIO_WritePin(control_port, control_pins[i], GPIO_PIN_RESET);				/* " " " */
	}

}

/**
  * @brief  sets data pins, followed by RS and RW control pins with instruction code being sent to the LCD when the E pin is toggled
  * @param  4 bit data value
  * @param  RS value
  * @param  RW value
  * @retval None
  */
static void LCM1602a_Write4_Data_Single(uint8_t dataValues, uint8_t RS, uint8_t RW){
	HAL_GPIO_WritePin(data_port, data_pins[0], ((dataValues >> (0)) & 1));			/*write to data lines*/
	HAL_GPIO_WritePin(data_port, data_pins[1], ((dataValues >> (1)) & 1));			/* " " " */
	HAL_GPIO_WritePin(data_port, data_pins[2], ((dataValues >> (2)) & 1));			/* " " " */
	HAL_GPIO_WritePin(data_port, data_pins[3], ((dataValues >> (3)) & 1));			/* " " " */


	HAL_GPIO_WritePin(control_port, control_pins[0], RS);							/*write to control lines RS, RW*/
	HAL_GPIO_WritePin(control_port, control_pins[1], RW);							/* " " " */

	HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);					/*set E to High*/

	for(int j = CONTROL_PIN_COUNT - 1; j >= 0; j--){ 								/*reset all control pins*/
		HAL_GPIO_WritePin(control_port, control_pins[j], GPIO_PIN_RESET);			/* " " " */
	}
}

/**
  * @brief  sets data pins, followed by RS and RW control pins with instruction code being sent to the LCD when the E pin is toggled
  * @param  8 bit data value
  * @param  RS value
  * @param  RW value
  * @retval None
  */
static void LCM1602a_Write4_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	for(int i = 1; i >= 0; i--){

		LCM1602a_Hang_Busy_Flag();														/*hang until busy flag is reset*/

		HAL_GPIO_WritePin(data_port, data_pins[0], ((dataValues >> (0 + 4*i)) & 1));	/*write to data lines*/
		HAL_GPIO_WritePin(data_port, data_pins[1], ((dataValues >> (1 + 4*i)) & 1));	/* " " " */
		HAL_GPIO_WritePin(data_port, data_pins[2], ((dataValues >> (2 + 4*i)) & 1));	/* " " " */
		HAL_GPIO_WritePin(data_port, data_pins[3], ((dataValues >> (3 + 4*i)) & 1));	/* " " " */


		HAL_GPIO_WritePin(control_port, control_pins[0], RS);							/*write to control lines RS, RW*/
		HAL_GPIO_WritePin(control_port, control_pins[1], RW);							/* " " " */

		HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);					/*set E to High*/

		for(int j = CONTROL_PIN_COUNT - 1; j >= 0; j--){ 								/*reset all control pins*/
			HAL_GPIO_WritePin(control_port, control_pins[j], GPIO_PIN_RESET);			/* " " " */
		}
	}

}

/**
  * @brief  Hangs until the Data 7 pin returns the LOW value (Busy Flag)
  * @param  None
  * @retval None
  */
static void LCM1602a_Hang_Busy_Flag(){

	data_port->MODER &= ~(GPIO_PIN_TO_MODER(data_7_pin));								/*set D7 to input*/

	while(1){																			/*hang till Busy flag is Low*/

		HAL_GPIO_WritePin(control_port, control_pins[1], GPIO_PIN_SET);					/*set RW and E*/
		HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);					/* " " " */

		if(HAL_GPIO_ReadPin(data_port, data_pins[7]) == 0){								/*if Data 7 pin is 0*/
			HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_RESET);				/*set E back to LOW and stop hanging*/
			break;
		}
	}

	data_port->MODER |= GPIO_PIN_TO_MODER_0(data_7_pin);								/*set D7 to output*/
}
