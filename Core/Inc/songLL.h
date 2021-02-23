/*
 * song_linked_list.h
 *
 *  Created on: Nov 18, 2020
 *      Author: Rami
 */

#ifndef INC_SONGLL_H_
#define INC_SONGLL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct _mp3file{
	char *pcMp3Name;
	struct _mp3file *pxNext;
	struct _mp3file *pxPrev;
}MP3;

MP3 *pxSongLLNewElement(char* pcFileName);
void vSongLLAddEnd(MP3 **ppxHead, MP3 *pxNewPointer);
void vSongLLCircularizeList(MP3 *pxHead);
void vSongLLPrintList(MP3 **head);



#endif /* INC_SONGLL_H_ */
