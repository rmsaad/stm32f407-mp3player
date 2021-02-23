/*
 * mp3player.h
 *
 *  Created on: Oct 8, 2020
 *      Author: Rami
 */

#ifndef INC_MP3PLAYER_H_
#define INC_MP3PLAYER_H_

#include<stdint.h>

typedef enum{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

/* Public function prototypes -----------------------------------------------*/
void vMp3PlayerInit();
void vMp3PlayerFindInfo();
void vMp3PlayerDecodeFrames();

#endif /* INC_MP3PLAYER_H_ */
