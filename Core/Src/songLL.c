/*
 * song_linked_list.c
 *
 *  Created on: Nov 18, 2020
 *      Author: Rami
 */

#include <songLL.h>

/**
  * @brief  create and allocate memory for new node in LL
  * @param  pcFileName : Name of LL node
  * @retval MP3 Node variable
  */
MP3 *pxSongLLNewElement(char* pcFileName){
    MP3 *pxNewPointer;
    pxNewPointer = (MP3 *) malloc(sizeof(MP3));
    pxNewPointer->pcMp3Name = (char *) malloc(strlen(pcFileName) + 1);
    strcpy(pxNewPointer->pcMp3Name, pcFileName);
    pxNewPointer->pxNext = NULL;
    pxNewPointer->pxPrev = NULL;
    return pxNewPointer;
}

/**
  * @brief  Add node to end of LL
  * @param  ppxHead      : pointer to pointer of head of LL
  * @param  pxNewPointer : pointer to new node
  * @retval None
  */
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

/**
  * @brief  Connect Beginning and End of LL together
  * @param  pxHead : pointer to head of LL
  * @retval None
  */
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

/**
  * @brief  Prints out all elements of LL
  * @param  ppxHead : pointer to pointer of head of LL
  * @retval None
  */
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

