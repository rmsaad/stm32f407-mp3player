/*
 * lcm1602a_driver.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Rami
 */

#include <string.h>
#include "main.h"

GPIO_InitTypeDef GPIO_InitStruct = {0};

void LCM1602a_Hang_Busy_Flag();

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

/**
 * @fn			: LCM1602a_Hang_Busy_Flag
 *
 * @brief		: Hangs until the Data 7 pin returns the LOW value (Busy Flag)
 *
 *
 * @return		: none
 */
void LCM1602a_Hang_Busy_Flag(){

	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}
