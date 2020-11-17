/*
 * lcm1602a_driver.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Rami
 */

#include <string.h>
#include "main.h"

GPIO_InitTypeDef GPIO_InitStruct = {0};

uint8_t transmission_mode;

void LCM1602a_Hang_Busy_Flag();

void LCM1602a_8bit_init(GPIO_TypeDef *dataports[8], uint16_t dataPins[8]){
	transmission_mode = DATA_8;

	for(int i = 0; i < DATA_8; i++){

	}

	/*Initialize the display 8 bit mode*/
	LCM1602a_Write8_Data(0b00111000, 0, 0);
	LCM1602a_Write8_Data(0b00001110, 0, 0);
	LCM1602a_Write8_Data(0b00000110, 0, 0);

	/*clear the display*/
	LCM1602a_Write8_Data(0b00000001, 0, 0);

}

void LCM1602a_4bit_init(GPIO_TypeDef *dataports[4], uint16_t dataPins[4]){
	transmission_mode = DATA_4;

	for(int i = 0; i < DATA_4; i++){

	}
}

/**
 * @fn			: LCM1602a_Write8_Message
 *
 * @brief		: Writes a char array to the LCD screen
 *
 * @param[in]	: char pointer to the ascii message to be displayed on the LCD screen
 *
 * @return		: none
 */
void LCM1602a_Write8_Message(char *Message){

	uint16_t Len = (uint16_t)strlen(Message);											/*find length of the message*/

	for(int i = 0; i < Len; i++){														/*write message to display*/
		LCM1602a_Write8_Data((int)*Message, 1, 0);
		Message++;
	}
}

void LCM1602a_Write4_Message(char *Message){

	uint16_t Len = (uint16_t)strlen(Message);											/*find length of the message*/

	for(int i = 0; i < Len; i++){														/*write message to display*/
		LCM1602a_Write4_Data((int)*Message, 1, 0);
		Message++;
	}
}

/**
 * @fn			: LCM1602a_Write8_Data
 *
 * @brief		: sets data pins, followed by RS and RW control pins with instruction code being sent to the LCD when the E pin is toggled
 *
 * @param[in]	: data values to be written to data pins with Data7 being MSB and Data0 being LSB
 * @param[in]	: RS control pin value
 * @param[in]	: RW control pin value
 *
 * @return		: none
 */
void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	LCM1602a_Hang_Busy_Flag();															/*hang until busy flag is reset*/

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,  ((dataValues >> 0) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,  ((dataValues >> 1) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  ((dataValues >> 2) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, ((dataValues >> 3) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, ((dataValues >> 4) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, ((dataValues >> 5) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, ((dataValues >> 6) & 1));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, ((dataValues >> 7) & 1));
																						/*write to control lines RS, RW*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RS);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RW);

																						/*set E to High*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

/*this is not working 100% yet*/
void LCM1602a_Write4_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	for(int i = 1; i >= 0; i--){

		LCM1602a_Hang_Busy_Flag();														/*hang until busy flag is reset*/

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, ((dataValues >> (0 + 4*i)) & 1));
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, ((dataValues >> (1 + 4*i)) & 1));
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, ((dataValues >> (2 + 4*i)) & 1));
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, ((dataValues >> (3 + 4*i)) & 1));

																						/*write to control lines RS, RW*/
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RS);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RW);


																						/*set E to High*/
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	}


}

/**
 * @fn			: LCM1602a_Hang_Busy_Flag
 *
 * @brief		: Hangs until the Data 7 pin returns the LOW value (Busy Flag)
 *
 *
 * @return		: none
 */
void LCM1602a_Hang_Busy_Flag(){

	GPIOE->MODER &= ~(GPIO_MODER_MODER14);												/*set D7 to input*/

	while(1){																			/*hang till Busy flag is Low*/
																						/*set RW and E*/
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
																						/*read Data 7 pin, if 0 set E back to LOW and stop hanging*/
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		}
	}

	GPIOE->MODER |= GPIO_MODER_MODE14_0;												/*set D7 to output*/
}
