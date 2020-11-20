/*
 * song_linked_list.h
 *
 *  Created on: Nov 18, 2020
 *      Author: Rami
 */

#ifndef INC_SONG_LINKED_LIST_H_
#define INC_SONG_LINKED_LIST_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct _mp3file{
	char *mp3name;
	struct _mp3file *next;
	struct _mp3file *prev;
}MP3;

MP3 *newelement(char* fname);
void addend(MP3 **head, MP3 *new_p);
void circularize_list(MP3 *head);
void printlist(MP3 **head);



#endif /* INC_SONG_LINKED_LIST_H_ */
