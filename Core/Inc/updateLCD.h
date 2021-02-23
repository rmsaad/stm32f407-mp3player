/*
 * updateLCD.h
 *
 *  Created on: Feb 10, 2021
 *      Author: Rami
 */

#ifndef INC_UPDATELCD_H_
#define INC_UPDATELCD_H_

typedef struct{
	uint32_t ulCurrentTime;
	uint32_t ulTotalTime;
	uint32_t ulVolume;
	uint32_t ulSampleRate;
	char cCurrentTime[12];
	char cTotalTime[12];
	char cCurrentVolume[6];
	char cMp3Track[50];
}DisplayInfoTypeDef;


/* Public function prototypes -----------------------------------------------*/

/*Getters and Setters*/
void vUpdateLCDSetCurrentTime(uint32_t ulSetToCurrentTime);
uint32_t ulUpdateLCDGetCurrentTime();
void vUpdateLCDSetTotalTime(uint32_t ulSetToTotalTime);
void vUpdateLCDSetVolume(uint32_t ulSetToVolume);
uint32_t ulUpdateLCDGetVolume();
void vUpdateLCDSetSampleRate(uint32_t ulSetToSampleRate);
uint32_t ulUpdateLCDGetSampleRate();
void vUpdateLCDSetMp3Track(const char * pcMp3Track);

/*others*/
void vUpdateLCDScreen();


#endif /* INC_UPDATELCD_H_ */
