/*
 * updateLCD.c
 *
 *  Created on: Feb 10, 2021
 *      Author: Rami
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/

/*Display variables*/
static DisplayInfoTypeDef xDisplayInfo = {0, 0, 0, 0, "", "", "", ""};

/* Private function prototypes -----------------------------------------------*/
static void prvUpdateLCDConvertToMinutes(uint32_t ulSeconds, char cTimeString[12]);
static void prvUpdateLCDPrintVolume();

/**
  * @brief  Set current time LCD variable
  * @param  ulSetToCurrentTime: variable to set the current time on LCD.
  * @retval None
  */
void vUpdateLCDSetCurrentTime(uint32_t ulSetToCurrentTime){
	xDisplayInfo.ulCurrentTime = ulSetToCurrentTime;
}

/**
  * @brief  returns current time LCD variable
  * @retval ""
  */
uint32_t ulUpdateLCDGetCurrentTime(){
	return xDisplayInfo.ulCurrentTime;
}

/**
  * @brief  Set total time LCD variable, convert it from integer to "mm:ss" string
  * @param  ulSetToCurrentTime: variable to set the total time on LCD.
  * @retval None
  */
void vUpdateLCDSetTotalTime(uint32_t ulSetToTotalTime){

	xDisplayInfo.ulTotalTime = ulSetToTotalTime;

	if(ulSetToTotalTime != 0){
		prvUpdateLCDConvertToMinutes(xDisplayInfo.ulTotalTime, xDisplayInfo.cTotalTime);
	}else{
		strncpy(xDisplayInfo.cTotalTime , "--:--", 12);
	}
}

/**
  * @brief  Set volume LCD variable
  * @param  ulSetToCurrentTime: variable to set the volume variable for LCD.
  * @retval None
  */
void vUpdateLCDSetVolume(uint32_t ulSetToVolume){
	xDisplayInfo.ulVolume = ulSetToVolume;
}

/**
  * @brief  returns Volume LCD variable
  * @retval ""
  */
uint32_t ulUpdateLCDGetVolume(){
	return xDisplayInfo.ulVolume;
}

/**
  * @brief  Set Sample rate variable
  * @param  ulSetToCurrentTime: variable to set Sample rate for playback use
  * @retval None
  */
void vUpdateLCDSetSampleRate(uint32_t ulSetToSampleRate){
	xDisplayInfo.ulSampleRate = ulSetToSampleRate;
}

/**
  * @brief  returns current tracks sample rate
  * @retval ""
  */
uint32_t ulUpdateLCDGetSampleRate(){
	return xDisplayInfo.ulSampleRate;
}

/**
  * @brief  Set Mp3 Track LCD string
  * @param  ulSetToCurrentTime: String to appear as Mp3 track on LCD display
  * @retval None
  */
void vUpdateLCDSetMp3Track(const char * pcMp3Track){
	strncpy(xDisplayInfo.cMp3Track, pcMp3Track, 50);
}

/**
  * @brief  Convert seconds from integer to char[], in format of mm:ss
  * @param  seconds		:	time in seconds
  * @param  time_string	:	time in mm:ss char format
  * @retval None
  */
static void prvUpdateLCDConvertToMinutes(uint32_t ulSeconds, char cTimeString[12]){
	uint32_t minutes = ulSeconds/60;																								/*minutes calculation*/
	ulSeconds = ulSeconds - ((ulSeconds/60) * 60);																					/*seconds calculation*/
	sprintf(cTimeString, "%02ld:%02ld", minutes, ulSeconds);																		/*convert to string*/
}

/**
  * @brief  Displays current volume level to the display represented in bars
  * @param  None
  * @retval None
  */
static void prvUpdateLCDPrintVolume(){

	LCM1602a_Write_Data(SPEAKER, 1, 0);																								/*print speaker character*/

	if(xDisplayInfo.ulVolume < 25){																									/*if volume less than 25*/
		LCM1602a_Write_Data(VOLUME1, 1, 0);																								/* #--- */
		LCM1602a_Write_Message((char*) " ");
	}else if(xDisplayInfo.ulVolume < 50){																							/*else if volume less than 50*/
		LCM1602a_Write_Data(VOLUME2, 1, 0);																								/* ##-- */
		LCM1602a_Write_Message((char*) " ");
	}else if(xDisplayInfo.ulVolume < 75){																							/*else if volume less than 75*/
		LCM1602a_Write_Data(VOLUME2, 1, 0);																								/* ###- */
		LCM1602a_Write_Data(VOLUME3, 1, 0);
	}else if(xDisplayInfo.ulVolume <= 100){																							/*else if volume less than / equal to 100*/
		LCM1602a_Write_Data(VOLUME2, 1, 0);																								/* #### */
		LCM1602a_Write_Data(VOLUME4, 1, 0);
	}
}

/**
  * @brief  update the LCM1602a with current information
  * @param  None
  * @retval None
  */
void vUpdateLCDScreen(){
	prvUpdateLCDConvertToMinutes(xDisplayInfo.ulCurrentTime, xDisplayInfo.cCurrentTime);											/*convert current time to character string*/
	LCM1602a_Write_Data(0b00000010, 0, 0);																							/*Return to Home position on display*/
	LCM1602a_textwrap((char*) xDisplayInfo.cMp3Track, 1);
	LCM1602a_Write_Data(0b11000000, 0, 0);																							/*next line on display*/
	LCM1602a_Write_Message((char*) xDisplayInfo.cCurrentTime);																		/*display time information*/
	LCM1602a_Write_Message((char*) "/");																							/* "" "" "" */
	LCM1602a_Write_Message((char*) xDisplayInfo.cTotalTime);																		/* "" "" "" */
	LCM1602a_Write_Message((char*) " ");																							/*space character*/
	prvUpdateLCDPrintVolume();																										/*current volume*/
}

