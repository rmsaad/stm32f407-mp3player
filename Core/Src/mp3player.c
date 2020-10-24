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

#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#define WAVE_NAME "audio_sample.wav"
#define MP3_NAME1 "new1.raw"
#define MP3_NAME2 "out_ref.raw"
#define MP3_NAME3 "Innerspace.mp3"

/*MP3 BUFFER SIZES*/
#define MP3_BUF             (16 * 1024)
#define AUDIO_BUFFER_SIZE	9216

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

/*minimp3 decoding structs*/
static mp3dec_t mp3d;
static mp3dec_frame_info_t info;

/*encoded mp3 data read from USB storage*/
char input_data[MP3_BUF];

/*decoded audio pcm data*/
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/*mp3 play-back variables*/
int samples;
static __IO uint32_t AudioRemSize = 0;
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
static uint32_t MP3DataLength = 0;

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
  * @brief  fills the input buffer with mp3 data at the beginning of mp3 decoding
  * @param  None
  * @retval None
  */
void fill_first_input_buffer(uint32_t *bytesread){
	f_lseek(&FileRead, 0);																										/*seek to beginning of file*/
	f_read(&FileRead, &input_data[0], MP3_BUF, (void *) bytesread);																/*read mp3 data into input buffer*/
	AudioRemSize = MP3DataLength - *bytesread;																					/*calculate AudioRemSize*/
}

/**
  * @brief  Decodes a single frame of the mp3 file and stores the result in the Audio buffer
  * @param  None
  * @retval None
  */
void mp3_decode(uint32_t *bytesread, int bufferPos){

	samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, (short*) &Audio_Buffer[bufferPos], &info);  /*decode input data, store PCM in buffer*/
	memmove(&input_data[0], &input_data[info.frame_bytes], (MP3_BUF - info.frame_bytes));   									/*move input buffer to to correct position*/
	f_read(&FileRead, &input_data[MP3_BUF - info.frame_bytes], info.frame_bytes, (void *) bytesread); 							/*read more data into end of input buffer*/
	AudioRemSize = AudioRemSize - *bytesread;																					/*update AudioRemSize variable*/

	/*if (!*bytesread){
		int n = 0;
		while(info.frame_bytes){
			n += info.frame_bytes;
			samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[n], MP3_BUF - n, (short*) &Audio_Buffer[bufferPos], &info);
		}
	}*/

}

/**
  * @brief  Manages MP3 decoding process, and fills the Audio buffer when buffer offset flags are set
  * @param  samplerate	: mp3 file samplerate
  * @retval None
  */
void mp3_playback(uint32_t samplerate){
	uint32_t bytesread = 0;															/*variable to track bytes read*/
	if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, samplerate) != 0)					/* Initialize MP3 player (Audio Codec, DMA, I2C) */
	{
	Error_Handler();
	}

	fill_first_input_buffer(&bytesread);											/*fill input buffer completely input buffer*/
	mp3_decode(&bytesread, 0);														/*decode mp3 data, store result in first half of Audio Buffer*/
	mp3_decode(&bytesread, AUDIO_BUFFER_SIZE/2);									/*decode mp3 data, store result in second half of Audio Buffer*/

	BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE); 			/*start playing MP3*/

	while((AudioRemSize != 0)){														/*enter DMA play-back loop*/
		bytesread = 0;																/*set bytes read back to zero*/

		if(buffer_offset == BUFFER_OFFSET_HALF){									/*check if the first half of the Audio Buffer has been transferred*/
			mp3_decode(&bytesread, 0);													/*decode next mp3 data to replace it in the Audio buffer*/
			buffer_offset = BUFFER_OFFSET_NONE;											/*update buffer offset*/
		}

		if(buffer_offset == BUFFER_OFFSET_FULL){									/*check if the second half of the Audio Buffer has been transferred*/
			mp3_decode(&bytesread, AUDIO_BUFFER_SIZE/2);								/*decode next mp3 data to replace it in the Audio buffer*/
			buffer_offset = BUFFER_OFFSET_NONE;											/*update buffer offset*/
		}

		if(AudioRemSize > 0){
		// AudioRemSize -= bytesread;
		}
		else{
		AudioRemSize = 0;
		}
	}

}

/**
  * @brief  Opens mp3 file and calculates track length based on total samples
  * @param  None
  * @retval time of the current track being decoded
  */
uint32_t find_track_length(){
	uint32_t bytesread = 0, samplesCount = 0, bufPos = 0;
	while(1){
		bytesread = 0;
		samplesCount += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, 0, &info); 					/*count the number of samples*/
		memmove(&input_data[0], &input_data[info.frame_bytes], (MP3_BUF - info.frame_bytes));      							/*move input buffer to to correct position*/
		f_read(&FileRead, &input_data[MP3_BUF - info.frame_bytes], info.frame_bytes, (unsigned int*) &bytesread);  			/*read more data into end of input buffer*/

		if(bytesread == 0){																									/*if no more data can be read into the input buffer*/
			while(info.frame_bytes){																							/*while there are still mp3 frames in buffer*/
				bufPos += info.frame_bytes;																							/*track current frame position*/
				samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[bufPos], MP3_BUF - bufPos, 0, &info);			/*continue counting final samples in the buffer*/
			}
		}
		if (info.frame_bytes <= 0) break;																					/*break loop when there are no frames left*/
	}
	return samples;
}

/**
  * @brief  Opens mp3 file and initializes the mp3 decoding process
  * @param  None
  * @retval None
  */
void mp3player_start(void){
	char path[] = "0:/";
	char* mp3filename = NULL;
	/* Get the read out protection status */
	if(f_opendir(&Directory, path) == FR_OK)
	{

		mp3filename = MP3_NAME3;

		/* Open the Wave file to be played */
		if(f_open(&FileRead, mp3filename , FA_READ) != FR_OK)
		{
		  BSP_LED_On(LED5);
		}
		else
		{

			mp3dec_init(&mp3d);
			MP3DataLength = f_size(&FileRead);
			//find_track_length();
			samples = 0;
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
			mp3_playback(44100);

		}
	}

}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
	 buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	 buffer_offset = BUFFER_OFFSET_FULL;
	 BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}

  /* Could also generate a system reset to recover from the error */
  /* .... */
}
