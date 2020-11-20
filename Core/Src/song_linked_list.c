/*
 * song_linked_list.c
 *
 *  Created on: Nov 18, 2020
 *      Author: Rami
 */

#include "song_linked_list.h"

MP3 *newelement(char* fname){
	MP3 *new_p;
	new_p = (MP3 *) malloc(sizeof(MP3));
	new_p->mp3name = (char *) malloc(strlen(fname) + 1);
	strcpy(new_p->mp3name, fname);
	new_p->next = NULL;
	new_p->prev = NULL;
	return new_p;
}

void addend(MP3 **head, MP3 *new_p){
	MP3 *p2;
	if((*head) == NULL){
		*head = new_p;
	}else{
		for(p2 = *head; p2->next !=NULL; p2 = p2->next){
			;
		}

		p2->next = new_p;
		new_p->prev = p2;
	}
}

void circularize_list(MP3 *head){
	MP3 *p2;
	if(head == NULL)
		return;

	for(p2 = head; p2->next !=NULL; p2 = p2->next){
		;
	}

	head->prev = p2;
	p2->next = head;
}

void printlist(MP3 **head){
	MP3 **tracer = head;
	while ((*tracer) != NULL) {
		printf("%s \n",(*tracer)->mp3name);
		tracer = &(*tracer)->next;

		if((*tracer) == ((*head)->prev)){
			printf("%s \n",(*tracer)->mp3name);
			break;
		}

	}
}

