/*
 * mp3player.h
 *
 *  Created on: Oct 8, 2020
 *      Author: Rami
 */

#ifndef INC_MP3PLAYER_H_
#define INC_MP3PLAYER_H_

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

/*function prototypes*/
void mp3_playback(uint32_t samplerate);
void mp3player_start(void);


#endif /* INC_MP3PLAYER_H_ */
