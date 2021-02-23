/*
 * song_linked_list.c
 *
 *  Created on: Nov 18, 2020
 *      Author: Rami
 */

#include <songLL.h>

MP3 *pxSongLLNewElement(char* pcFileName){
	MP3 *pxNewPointer;
	pxNewPointer = (MP3 *) malloc(sizeof(MP3));
	pxNewPointer->pcMp3Name = (char *) malloc(strlen(pcFileName) + 1);
	strcpy(pxNewPointer->pcMp3Name, pcFileName);
	pxNewPointer->pxNext = NULL;
	pxNewPointer->pxPrev = NULL;
	return pxNewPointer;
}

void vSongLLAddEnd(MP3 **ppxHead, MP3 *pxNewPointer){
	MP3 *pxPointer2;
	if((*ppxHead) == NULL){
		*ppxHead = pxNewPointer;
	}else{
		for(pxPointer2 = *ppxHead; pxPointer2->pxNext !=NULL; pxPointer2 = pxPointer2->pxNext){
			;
		}

		pxPointer2->pxNext = pxNewPointer;
		pxNewPointer->pxPrev = pxPointer2;
	}
}

void vSongLLCircularizeList(MP3 *pxHead){
	MP3 *pxPointer2;
	if(pxHead == NULL)
		return;

	for(pxPointer2 = pxHead; pxPointer2->pxNext !=NULL; pxPointer2 = pxPointer2->pxNext){
		;
	}

	pxHead->pxPrev = pxPointer2;
	pxPointer2->pxNext = pxHead;
}

void vSongLLPrintList(MP3 **ppxHead){
	MP3 **ppxTracer = ppxHead;
	while ((*ppxTracer) != NULL) {
		printf("%s \n",(*ppxTracer)->pcMp3Name);
		ppxTracer = &(*ppxTracer)->pxNext;

		if((*ppxTracer) == ((*ppxHead)->pxPrev)){
			printf("%s \n",(*ppxTracer)->pcMp3Name);
			break;
		}

	}
}

