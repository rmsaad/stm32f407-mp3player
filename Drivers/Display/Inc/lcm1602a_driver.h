/*
 * lcm1602a_driver.h
 *
 *  Created on: Oct 31, 2020
 *      Author: Rami
 */

#ifndef DISPLAY_INC_LCM1602A_DRIVER_H_
#define DISPLAY_INC_LCM1602A_DRIVER_H_

/******************************************************************************/
/**************************** Generic Macros **********************************/

#define DATA_8					8
#define DATA_4					4
#define CONTROL_PIN_COUNT		3

void LCM1602a_Write8_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
void LCM1602a_Write8_Message(char *Message);

void LCM1602a_Write4_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
void LCM1602a_Write4_Message(char *Message);

#endif /* DISPLAY_INC_LCM1602A_DRIVER_H_ */
