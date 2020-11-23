/*
 * lcm1602a_driver.h
 *
 *  Created on: Oct 31, 2020
 *      Author: Rami
 */

#ifndef DISPLAY_INC_LCM1602A_DRIVER_H_
#define DISPLAY_INC_LCM1602A_DRIVER_H_



/* Generic define ------------------------------------------------------------*/

#define DATA_8					8
#define DATA_4					4
#define CONTROL_PIN_COUNT		3

#define ONE_LINE_DISPLAY		1
#define TWO_LINE_DISPLAY		2

#define SPEAKER					0
#define VOLUME1					1
#define VOLUME2					2
#define VOLUME3					3
#define VOLUME4					4

/* Generic macro -------------------------------------------------------------*/

#define GPIO_PIN_TO_MODER(x)	((x == GPIO_PIN_0)  ? (0x3UL << (0U))  :\
								 (x == GPIO_PIN_1)  ? (0x3UL << (2U))  :\
								 (x == GPIO_PIN_2)  ? (0x3UL << (4U))  :\
								 (x == GPIO_PIN_3)  ? (0x3UL << (6U))  :\
								 (x == GPIO_PIN_4)  ? (0x3UL << (8U))  :\
								 (x == GPIO_PIN_5)  ? (0x3UL << (10U)) :\
								 (x == GPIO_PIN_6)  ? (0x3UL << (12U)) :\
								 (x == GPIO_PIN_7)  ? (0x3UL << (14U)) :\
								 (x == GPIO_PIN_8)  ? (0x3UL << (16U)) :\
								 (x == GPIO_PIN_9)  ? (0x3UL << (18U)) :\
								 (x == GPIO_PIN_10) ? (0x3UL << (20U)) :\
								 (x == GPIO_PIN_11) ? (0x3UL << (22U)) :\
								 (x == GPIO_PIN_12) ? (0x3UL << (24U)) :\
								 (x == GPIO_PIN_13) ? (0x3UL << (26U)) :\
								 (x == GPIO_PIN_14) ? (0x3UL << (28U)) :\
								 (x == GPIO_PIN_15) ? (0x3UL << (30U)) :0)

#define GPIO_PIN_TO_MODER_0(x)	((x == GPIO_PIN_0)  ? (0x1UL << (0U))  :\
								 (x == GPIO_PIN_1)  ? (0x1UL << (2U))  :\
								 (x == GPIO_PIN_2)  ? (0x1UL << (4U))  :\
								 (x == GPIO_PIN_3)  ? (0x1UL << (6U))  :\
								 (x == GPIO_PIN_4)  ? (0x1UL << (8U))  :\
								 (x == GPIO_PIN_5)  ? (0x1UL << (10U)) :\
								 (x == GPIO_PIN_6)  ? (0x1UL << (12U)) :\
								 (x == GPIO_PIN_7)  ? (0x1UL << (14U)) :\
								 (x == GPIO_PIN_8)  ? (0x1UL << (16U)) :\
								 (x == GPIO_PIN_9)  ? (0x1UL << (18U)) :\
								 (x == GPIO_PIN_10) ? (0x1UL << (20U)) :\
								 (x == GPIO_PIN_11) ? (0x1UL << (22U)) :\
								 (x == GPIO_PIN_12) ? (0x1UL << (24U)) :\
								 (x == GPIO_PIN_13) ? (0x1UL << (26U)) :\
								 (x == GPIO_PIN_14) ? (0x1UL << (28U)) :\
								 (x == GPIO_PIN_15) ? (0x1UL << (30U)) :0)


/* Public function prototypes -----------------------------------------------*/

void LCM1602a_Set_DATA8(GPIO_TypeDef *d_Port, uint16_t d_Pins[8], GPIO_TypeDef *c_Port, uint16_t c_Pins[3]);
void LCM1602a_Set_DATA4(GPIO_TypeDef *d_Port, uint16_t d_Pins[4], GPIO_TypeDef *c_Port, uint16_t c_Pins[3]);

void LCM1602a_init(uint8_t disp_line);

void LCM1602a_textwrap(char* text);
void LCM1602a_Write_Data(uint8_t dataValues, uint8_t RS, uint8_t RW);
void LCM1602a_Write_Message(char *Message);


#endif /* DISPLAY_INC_LCM1602A_DRIVER_H_ */
