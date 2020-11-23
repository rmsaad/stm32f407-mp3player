/*
 * lcm1602a_driver.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Rami
 */

#include <string.h>
#include "main.h"

GPIO_InitTypeDef GPIO_InitStruct = {0};

GPIO_TypeDef *data_port;
GPIO_TypeDef *control_port;

uint16_t data_pins[8];
uint16_t control_pins[3];

uint8_t transmission_mode;

/*function prototypes*/
static void LCM1602a_Hang_Busy_Flag();

void LCM1602a_Set_DATA8(GPIO_TypeDef *d_Port, uint16_t d_Pins[8], GPIO_TypeDef *c_Port, uint16_t c_Pins[3]){

	transmission_mode = DATA_8;
	data_port = d_Port;
	control_port = c_Port;

	for(int i = 0; i < DATA_8; i++){
		data_pins[i] = d_Pins[i];
	}

	for(int i = 0; i < CONTROL_PIN_COUNT; i++){
		control_pins[i] = c_Pins[i];
	}

}

void LCM1602a_Set_DATA4(GPIO_TypeDef *d_Port, uint16_t d_Pins[4], GPIO_TypeDef *c_Port, uint16_t c_Pins[3]){

	transmission_mode = DATA_4;
	data_port = d_Port;
	control_port = c_Port;

	for(int i = 0; i < DATA_4; i++){
		data_pins[i] = d_Pins[i];
	}

	for(int i = 0; i < CONTROL_PIN_COUNT; i++){
		control_pins[i] = c_Pins[i];
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

	for(int i = 0; i < DATA_8; i++){													/*write to data lines*/
		HAL_GPIO_WritePin(data_port, data_pins[i] , ((dataValues >> i) & 1));
	}
																						/*write to control lines RS, RW*/
	HAL_GPIO_WritePin(control_port, control_pins[0], RS);
	HAL_GPIO_WritePin(control_port, control_pins[1], RW);
																						/*set E to High*/
	HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);

	for(int i = CONTROL_PIN_COUNT - 1; i >= 0; i--){ 									/*reset all control pins*/
		HAL_GPIO_WritePin(control_port, control_pins[i], GPIO_PIN_RESET);
	}

}

/*this is not working 100% yet*/
void LCM1602a_Write4_Data(uint8_t dataValues, uint8_t RS, uint8_t RW){

	for(int i = 1; i >= 0; i--){

		LCM1602a_Hang_Busy_Flag();														/*hang until busy flag is reset*/

		HAL_GPIO_WritePin(data_port, data_pins[4], ((dataValues >> (0 + 4*i)) & 1));
		HAL_GPIO_WritePin(data_port, data_pins[5], ((dataValues >> (1 + 4*i)) & 1));
		HAL_GPIO_WritePin(data_port, data_pins[6], ((dataValues >> (2 + 4*i)) & 1));
		HAL_GPIO_WritePin(data_port, data_pins[7], ((dataValues >> (3 + 4*i)) & 1));

																						/*write to control lines RS, RW*/
		HAL_GPIO_WritePin(control_port, control_pins[0], RS);
		HAL_GPIO_WritePin(control_port, control_pins[1], RW);
																						/*set E to High*/
		HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);

		for(int i = CONTROL_PIN_COUNT - 1; i >= 0; i--){ 								/*reset all control pins*/
			HAL_GPIO_WritePin(control_port, control_pins[i], GPIO_PIN_RESET);
		}
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

	data_port->MODER &= ~(GPIO_MODER_MODER14);											/*set D7 to input*/

	while(1){																			/*hang till Busy flag is Low*/
																						/*set RW and E*/
		HAL_GPIO_WritePin(control_port, control_pins[1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_SET);
																						/*read Data 7 pin, if 0 set E back to LOW and stop hanging*/
		if(HAL_GPIO_ReadPin(data_port, data_pins[7]) == 0){
			HAL_GPIO_WritePin(control_port, control_pins[2], GPIO_PIN_RESET);
			break;
		}
	}

	data_port->MODER |= GPIO_MODER_MODE14_0;											/*set D7 to output*/
}
