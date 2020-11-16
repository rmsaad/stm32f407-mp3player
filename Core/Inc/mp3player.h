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

typedef struct{
	uint32_t current_time;
	uint32_t total_time;
	uint32_t volume;
	uint32_t sample_rate;
	char cur_time[12];
	char tot_time[12];
	char cur_volume[6];
	char song_name[10];
	char artist_name[10];
}DisplayInfoTypeDef;

/*function prototypes*/
void mp3_playback(uint32_t samplerate);
void mp3player_start(void);
void update_volume();

#endif /* INC_MP3PLAYER_H_ */
