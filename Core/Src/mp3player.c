/*
 * mp3player.c
 *
 *  Created on: Oct 8, 2020
 *      Author: Rami
 */

#include "main.h"
#include "ff.h"
#include "string.h"
#include "stm32f4_discovery_audio.h"

#define MINIMP3_NO_STDIO
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#include "minimp3_ex.h"

#define WAVE_NAME "audio_sample.wav"
#define MP3_NAME1 "new1.raw"
#define MP3_NAME2 "out_ref.raw"
#define MP3_NAME3 "Innerspace.mp3"
#define MP3_NAME4 "test.mp3"

#define REMAP(X) (((X - OLD_MIN) * (NEW_MAX - NEW_MIN)) / (OLD_MAX - OLD_MIN)) + NEW_MIN

/*MP3 BUFFER SIZES*/
#define MP3_BUF             (16 * 1024)
#define AUDIO_BUFFER_SIZE	( 8 * 1152)

/*FatFs variables*/
FIL FileRead;
DIR Directory;

/*minimp3 decoding structs*/
static mp3dec_t mp3d;
static mp3dec_frame_info_t info;

/*encoded mp3 data read from USB storage*/
char input_data[MP3_BUF];

/*decoded audio pcm data*/
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/*Display variables*/
DisplayInfoTypeDef display_info = {0, 0, 0, 0, "", ""};

/*mp3 play-back variables*/
extern ADC_HandleTypeDef hadc1;
uint64_t samples;
uint8_t decodingfinished = 1;
static __IO uint32_t AudioRemSize = 0;
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
static uint32_t MP3DataLength = 0;


/**
  * @brief  Convert seconds from integer to char[], in format of mm:ss
  * @param  seconds		:	time in seconds
  * @param  time_string	:	time in mm:ss char format
  * @retval None
  */
void convert_to_minutes(uint32_t seconds, char time_string[12]){
	uint32_t minutes = seconds/60;																											/*minutes calculation*/
	seconds = seconds - ((seconds/60) * 60);																								/*seconds calculation*/
	sprintf(time_string, "%02ld:%02ld", minutes, seconds);																					/*convert to string*/
}

/**
  * @brief  polls potentiometer (w/ adc) and sets the volume based on its position
  * @param  None
  * @retval None
  */
void update_volume(){
	HAL_ADC_Start(&hadc1);																													/*start ADC conversion*/
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);																						/*poll ADC*/
	display_info.volume = REMAP(HAL_ADC_GetValue(&hadc1));																					/*get the ADC value*/
	BSP_AUDIO_OUT_SetVolume(display_info.volume);																							/*set volume appropriately*/
}

/**
  * @brief  print out current volume level to the display represented in bars
  * @param  None
  * @retval None
  */
void print_current_volume(){
	if(display_info.volume < 25){																											/*if volume less than 25*/
		LCM1602a_Write8_Data(0b11111111, 1, 0);																									/* #--- */
		LCM1602a_Write8_Message((char*) "   ");
	}else if(display_info.volume < 50){																										/*else if volume less than 50*/
		LCM1602a_Write8_Data(0b11111111, 1, 0);																									/* ##-- */
		LCM1602a_Write8_Data(0b11111111, 1, 0);
		LCM1602a_Write8_Message((char*) "  ");
	}else if(display_info.volume < 75){																										/*else if volume less than 75*/
		LCM1602a_Write8_Data(0b11111111, 1, 0);																									/* ###- */
		LCM1602a_Write8_Data(0b11111111, 1, 0);
		LCM1602a_Write8_Data(0b11111111, 1, 0);
		LCM1602a_Write8_Message((char*) " ");
	}else if(display_info.volume <= 100){																									/*else if volume less than / equal to 100*/
		LCM1602a_Write8_Data(0b11111111, 1, 0);																									/* #### */
		LCM1602a_Write8_Data(0b11111111, 1, 0);
		LCM1602a_Write8_Data(0b11111111, 1, 0);
		LCM1602a_Write8_Data(0b11111111, 1, 0);
	}
}

/**
  * @brief  update the LCM1602a with current information
  * @param  None
  * @retval None
  */
void update_display(){
	convert_to_minutes(display_info.current_time, display_info.cur_time);																	/*convert current time to character string*/
	LCM1602a_Write8_Data(0b00000010, 0, 0);																									/*Return to Home position on display*/
	LCM1602a_Write8_Message((char*) MP3_NAME3);
	LCM1602a_Write8_Data(0b11000000, 0, 0);																									/*next line on display*/
	LCM1602a_Write8_Message((char*) display_info.cur_time);																					/*display time information*/
	LCM1602a_Write8_Message((char*) "/");																									/* "" "" "" */
	LCM1602a_Write8_Message((char*) display_info.tot_time);																					/* "" "" "" */
	LCM1602a_Write8_Message((char*) " ");																									/*space character*/
	print_current_volume();																													/*current volume*/
}

/**
  * @brief  fills the input buffer with mp3 data at the beginning of mp3 decoding
  * @param  bytesread : amount of f_read bytes read
  * @retval None
  */
void fill_first_input_buffer(uint32_t *bytesread){
	f_lseek(&FileRead, 0);																													/*seek to beginning of file*/
	f_read(&FileRead, &input_data[0], MP3_BUF, (void *) bytesread);																			/*read mp3 data into input buffer*/
	AudioRemSize = MP3DataLength - *bytesread;																								/*calculate AudioRemSize*/
}

/**
  * @brief  Decodes a single frame of the mp3 file and stores the result in the Audio buffer
  * @param  bytesread : amount of f_read bytes read
  * @param  inBufPos  : position to set the Audio_Buffer
  * @retval None
  */
void mp3_decode(uint32_t *bytesread, int inBufPos){

	if (!AudioRemSize){																														/*finish decoding last buffer*/
		static uint32_t n = 0;
		if(info.frame_bytes){																												/*if there are still more frames left*/
			n += info.frame_bytes;																												/*track frame position in input buffer*/
			samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[n], MP3_BUF - n, (short*) &Audio_Buffer[inBufPos], &info);		/*decode input data, store PCM in buffer*/
		}else{																																/*if there are no frames left*/
			decodingfinished = 0;																												/*update mp3 decoding flag*/
		}
	}else{
		samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, (short*) &Audio_Buffer[inBufPos], &info);  			/*decode input data, store PCM in buffer*/
		memmove(&input_data[0], &input_data[info.frame_bytes], (MP3_BUF - info.frame_bytes));   											/*move input buffer to to correct position*/
		f_read(&FileRead, &input_data[MP3_BUF - info.frame_bytes], info.frame_bytes, (void *) bytesread); 									/*read more data into end of input buffer*/
		AudioRemSize = AudioRemSize - *bytesread;																							/*update AudioRemSize variable*/
	}
}

/**
  * @brief  Manages MP3 decoding process, and fills the Audio buffer when buffer offset flags are set
  * @param  samplerate	: mp3 file samplerate
  * @retval None
  */
void mp3_playback(uint32_t samplerate){
	uint32_t bytesread = 0;																													/*variable to track bytes read*/
	display_info.current_time = 0;																											/*initialize current mp3 time equal to 0*/
	uint32_t old_time = 0;																													/*initialize old mp3 time equal to 0*/
	mp3dec_init(&mp3d);																														/*start mininmp3 decoding process*/
	if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, samplerate) != 0){																		/* Initialize MP3 player (Codec, DMA, I2C) */
		Error_Handler();																														/*error if mp3 player initialization fails*/
	}

	fill_first_input_buffer(&bytesread);																									/*fill input buffer completely input buffer*/
	mp3_decode(&bytesread, 0);																												/*decode mp3 data, store result in first half of Audio Buffer*/
	mp3_decode(&bytesread, AUDIO_BUFFER_SIZE/2);																							/*decode mp3 data, store result in second half of Audio Buffer*/

	BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE); 																	/*start playing MP3*/

	while(decodingfinished){																												/*enter DMA play-back loop*/
		bytesread = 0;																														/*set bytes read back to zero*/
		display_info.current_time = samples / display_info.sample_rate;																		/*update current time*/
		if(display_info.current_time != old_time){																							/*if current time does not match old time*/
			update_display();																													/*update display*/
			old_time = display_info.current_time;																								/*set old time equal to current time*/
		}

		if(buffer_offset == BUFFER_OFFSET_HALF){																							/*check if the first half of the Audio Buffer has been transferred*/
			mp3_decode(&bytesread, 0);																											/*decode next mp3 data to replace it in the Audio buffer*/
			buffer_offset = BUFFER_OFFSET_NONE;																									/*update buffer offset*/
		}

		if(buffer_offset == BUFFER_OFFSET_FULL){																							/*check if the second half of the Audio Buffer has been transferred*/
			mp3_decode(&bytesread, AUDIO_BUFFER_SIZE/2);																						/*decode next mp3 data to replace it in the Audio buffer*/
			buffer_offset = BUFFER_OFFSET_NONE;																									/*update buffer offset*/
		}

		update_volume();																													/*update adc volume*/
	}

	 BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);																										/*stop audio play-back*/
	 f_close(&FileRead);																													/*close file*/
	 decodingfinished = 1;																													/*initialize decoding flag back to default*/
}

/**
  * @brief  custom minimp3 io read function
  * @param  f_read buffer
  * @param  data size in bytes
  * @param  mp3 file pointer
  * @retval bytes read
  */
size_t minimp3_io_read(void* buf, size_t size, void* user_data) {
	unsigned int br;
	f_read((FIL*) user_data, buf, (unsigned int) size, (unsigned int*) &br);
    return (size_t) br;
}

/**
  * @brief  custom minimp3 io seek function
  * @param  seek position
  * @param  mp3 file pointer
  * @retval FRESULT code
  */
static int minimp3_io_seek(uint64_t position, void* user_data){
	return f_lseek((FIL*) user_data, position);
}

/**
  * @brief  finds the track length and sample rate of the mp3
  * @param  None
  * @retval None
  */
static void minimp3_find_info(){

	mp3dec_ex_t dec;																														/*necessary minimp3 variables*/
	mp3dec_io_t io;																															/* "" "" "" */
	io.read = minimp3_io_read;																												/* "" "" "" */
	io.seek = minimp3_io_seek;																												/* "" "" "" */
	io.read_data = io.seek_data = &FileRead;																								/* "" "" "" */

	MP3DataLength = f_size(&FileRead);																										/*file length information*/

	if (mp3dec_ex_open_cb(&dec, &io, MP3D_SEEK_TO_SAMPLE)){																					/*find VBR tag*/
		Error_Handler();																														/*return error if mp3 cannot be parsed by minimp3*/
	}

	display_info.sample_rate = dec.info.hz;																									/*sample rate information*/

	if(dec.info.channels == 2){																												/*calculate mp3 track length*/
		display_info.total_time = dec.samples / (dec.info.hz * 2);																			/* "" "" "" */
		convert_to_minutes(display_info.total_time, display_info.tot_time);																	/* "" "" "" */
	}else{																																	/* "" "" "" */
		display_info.total_time = dec.samples / (dec.info.hz);																				/* "" "" "" */
		convert_to_minutes(display_info.total_time, display_info.tot_time);																	/* "" "" "" */
	}
	mp3dec_ex_close(&dec);																													/*free memory allocated by minimp3 library*/
}

/**
  * @brief  Opens mp3 file and initializes the mp3 decoding process
  * @param  None
  * @retval None
  */
void mp3player_start(void){
	char* mp3filename = MP3_NAME3;

	if(f_opendir(&Directory, "0:/") == FR_OK){																								/* Get the read out protection status */
		if(f_open(&FileRead, mp3filename , FA_READ) != FR_OK){																				/* Open the MP3 file to be played */
			Error_Handler();																													/*error if file does not exist*/

		}else{
			/*Initialize the display DATA 8 mode*/
			LCM1602a_Write8_Data(0b00111000, 0, 0);
			LCM1602a_Write8_Data(0b00001110, 0, 0);
			LCM1602a_Write8_Data(0b00000110, 0, 0);

			/*clear the display*/
			LCM1602a_Write8_Data(0b00000001, 0, 0);

			minimp3_find_info();																											/*retrieve mp3 information*/
			update_display();																												/*update display to reflect current info*/
			mp3_playback(display_info.sample_rate);																							/*start mp3 playback*/

		}
	}

}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
	 buffer_offset = BUFFER_OFFSET_HALF;																									/*set the buffer offset flag*/
}

/**
  * @brief  Calculates the remaining file size and new position of the pointer.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(void){
	 buffer_offset = BUFFER_OFFSET_FULL;																									/*set the buffer offset flag*/
	 BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);														/*fill buffer with next audio information*/
}

/**
  * @brief  Manages the DMA FIFO error interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(void){
  while (1){																																/*Stop the program with an infinite loop*/
	  Error_Handler();																														/*Go to error handler*/
  }
}

/**
  * @brief  LEGACY: Opens mp3 file and calculates track length in seconds based on total samples
  * @param  None
  * @retval time of the current track being decoded
  */
uint32_t find_track_length(){
	uint32_t bytesread = 0, samplesCount = 0, bufPos = 0;
	while(1){
		bytesread = 0;
		samplesCount += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, 0, &info); 									/*count the number of samples*/
		memmove(&input_data[0], &input_data[info.frame_bytes], (MP3_BUF - info.frame_bytes));      											/*move input buffer to to correct position*/
		f_read(&FileRead, &input_data[MP3_BUF - info.frame_bytes], info.frame_bytes, (unsigned int*) &bytesread);  							/*read more data into end of input buffer*/

		if(bytesread == 0){																													/*if no more data can be read into the input buffer*/
			while(info.frame_bytes){																											/*while there are still mp3 frames in buffer*/
				bufPos += info.frame_bytes;																										/*track current frame position*/
				samplesCount += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[bufPos], MP3_BUF - bufPos, 0, &info);					/*continue counting final samples in the buffer*/
			}
		}
		if (info.frame_bytes <= 0) break;																									/*break loop when there are no frames left*/
	}
	return samplesCount/info.hz;
}
