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
	uint32_t minutes = seconds/60;
	seconds = seconds - ((seconds/60) * 60);

	sprintf(time_string, "%02ld:%02ld", minutes, seconds);
}

void update_volume(){
	uint32_t adc_raw_val = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_raw_val = HAL_ADC_GetValue(&hadc1);
	BSP_AUDIO_OUT_SetVolume(REMAP(adc_raw_val));
}

/**
  * @brief  update the LCM1602a with current information
  * @param  None
  * @retval None
  */
void update_display(){
	convert_to_minutes(display_info.current_time, display_info.cur_time);																	/*convert current time to character string*/
	LCM1602a_Write8_Data(0b00000001, 0, 0);																									/*clear the display*/
	LCM1602a_Write8_Message((char*) MP3_NAME3);
	LCM1602a_Write8_Data(0b11000000, 0, 0);																									/*next line on display*/
	LCM1602a_Write8_Message((char*) display_info.cur_time);
	LCM1602a_Write8_Message((char*) "/");
	LCM1602a_Write8_Message((char*) display_info.tot_time);
	LCM1602a_Write8_Data(0b11111111, 1, 0);
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
	if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, samplerate) != 0){																		/* Initialize MP3 player (Codec, DMA, I2C) */
	Error_Handler();
	}

	fill_first_input_buffer(&bytesread);																									/*fill input buffer completely input buffer*/
	mp3_decode(&bytesread, 0);																												/*decode mp3 data, store result in first half of Audio Buffer*/
	mp3_decode(&bytesread, AUDIO_BUFFER_SIZE/2);																							/*decode mp3 data, store result in second half of Audio Buffer*/

	BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE); 																	/*start playing MP3*/
	display_info.current_time = 0;
	uint32_t old_time = 0;
	while(decodingfinished){																												/*enter DMA play-back loop*/

		bytesread = 0;																														/*set bytes read back to zero*/

		display_info.current_time = samples / display_info.sample_rate;

		if(display_info.current_time != old_time){
			update_display();
			old_time = display_info.current_time;
		}

		if(buffer_offset == BUFFER_OFFSET_HALF){																							/*check if the first half of the Audio Buffer has been transferred*/
			mp3_decode(&bytesread, 0);																											/*decode next mp3 data to replace it in the Audio buffer*/
			buffer_offset = BUFFER_OFFSET_NONE;																									/*update buffer offset*/
			update_volume();
		}

		if(buffer_offset == BUFFER_OFFSET_FULL){																							/*check if the second half of the Audio Buffer has been transferred*/
			mp3_decode(&bytesread, AUDIO_BUFFER_SIZE/2);																						/*decode next mp3 data to replace it in the Audio buffer*/
			buffer_offset = BUFFER_OFFSET_NONE;																									/*update buffer offset*/
		}
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

	mp3dec_ex_t dec;
	mp3dec_io_t io;
	io.read = minimp3_io_read;
	io.seek = minimp3_io_seek;
	io.read_data = io.seek_data = &FileRead;

	if (mp3dec_ex_open_cb(&dec, &io, MP3D_SEEK_TO_SAMPLE)){
		/* error */
	}

	display_info.sample_rate = dec.info.hz;

	if(dec.info.channels == 2){
		display_info.total_time = dec.samples / (dec.info.hz * 2);
		convert_to_minutes(display_info.total_time, display_info.tot_time);
	}else{
		display_info.total_time = dec.samples / (dec.info.hz);
		convert_to_minutes(display_info.total_time, display_info.tot_time);
	}

	mp3dec_ex_close(&dec);
}

/**
  * @brief  Opens mp3 file and initializes the mp3 decoding process
  * @param  None
  * @retval None
  */
void mp3player_start(void){
	char path[] = "0:/";
	char* mp3filename = MP3_NAME3;
	/* Get the read out protection status */
	if(f_opendir(&Directory, path) == FR_OK){

		/* Open the MP3 file to be played */
		if(f_open(&FileRead, mp3filename , FA_READ) != FR_OK){
			Error_Handler();
		}
		else{

			/*Initialize the display 8 bit mode*/
			LCM1602a_Write8_Data(0b00111000, 0, 0);
			LCM1602a_Write8_Data(0b00001110, 0, 0);
			LCM1602a_Write8_Data(0b00000110, 0, 0);

			/*clear the display*/
			LCM1602a_Write8_Data(0b00000001, 0, 0);

			minimp3_find_info();
			update_display();
			mp3dec_init(&mp3d);
			MP3DataLength = f_size(&FileRead);
			mp3_playback(44100);

		}
	}

}


/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
	 buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void){
	 buffer_offset = BUFFER_OFFSET_FULL;
	 BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void){

  /* Stop the program with an infinite loop */
  while (1)
  {}

  /* Could also generate a system reset to recover from the error */
  /* .... */
}

/**
  * @brief  LEGACY : plays a fully decoded .raw mp3 file
  * @param  None
  * @retval None
  */
void mp3_rawplayback(uint32_t samplerate){

  unsigned int bytesread = 0;
  /* Initialize Wave player (Codec, DMA, I2C) */
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, samplerate) != 0)
  {
	Error_Handler();
  }

  /* Get Data from USB Flash Disk */
  f_lseek(&FileRead, 0);
  f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
  AudioRemSize = MP3DataLength - bytesread;

  /* Start playing Wave */
  BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);

  /* Check if the device is connected.*/
  while((AudioRemSize != 0))
  {

	  bytesread = 0;

	  if(buffer_offset == BUFFER_OFFSET_HALF)
	  {

		f_read(&FileRead,
			   &Audio_Buffer[0],
			   AUDIO_BUFFER_SIZE/2,
			   (void *)&bytesread);

		  buffer_offset = BUFFER_OFFSET_NONE;
	  }

	  if(buffer_offset == BUFFER_OFFSET_FULL)
	  {
		f_read(&FileRead,
			   &Audio_Buffer[AUDIO_BUFFER_SIZE/2],
			   AUDIO_BUFFER_SIZE/2,
			   (void *)&bytesread);

		  buffer_offset = BUFFER_OFFSET_NONE;
	  }
	  if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2))
	  {
		AudioRemSize -= bytesread;
	  }
	  else
	  {
		AudioRemSize = 0;
	  }
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
