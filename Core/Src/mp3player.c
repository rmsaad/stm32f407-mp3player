/*
 * mp3player.c
 *
 *  Created on: Oct 9, 2020
 *      Author: Rami
 */

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

#define MP3_BUF             4096
#define AUDIO_BUFFER_SIZE	9216

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

static mp3dec_t mp3d;
static mp3dec_frame_info_t info;
int samples;

char input_data[MP3_BUF];
// changed to 16 bit from 8 bit
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

static __IO uint32_t AudioRemSize = 0;
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

static uint32_t MP3DataLength = 0;

void mp3_rawplayback(uint32_t samplerate){

  unsigned int bytesread = 0;
  //uint64_t a = file_length(&FileRead);
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

void fill_first_buffer(unsigned int *bytesread){
	f_lseek(&FileRead, 0);
	f_read(&FileRead, &input_data[0], MP3_BUF, (void *) bytesread);
	AudioRemSize = MP3DataLength - *bytesread;
}

void mp3_decode(unsigned int *bytesread, int pos){
	if(pos == 0){
		samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, (short*) &Audio_Buffer[0], &info);
	}else{
		samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, (short*) &Audio_Buffer[AUDIO_BUFFER_SIZE/2], &info);
	}
	memmove(&input_data[0], &input_data[info.frame_bytes], (MP3_BUF - info.frame_bytes));   /**/
	f_read(&FileRead, &input_data[MP3_BUF - info.frame_bytes], info.frame_bytes, (void *) bytesread);  /**/
}

void mp3_playback(uint32_t samplerate){

  unsigned int bytesread = 0;
  /* Initialize Wave player (Codec, DMA, I2C) */
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, samplerate) != 0)
  {
	Error_Handler();
  }

  /* Get Data from USB Flash Disk */
  fill_first_buffer(&bytesread);

  /*fill buffer*/
  mp3_decode(&bytesread, 0);
  mp3_decode(&bytesread, 1);

  /* Start playing Wave */
  BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);

  /* Check if the device is connected.*/
  while((AudioRemSize != 0))
  {

	  bytesread = 0;

	  if(buffer_offset == BUFFER_OFFSET_HALF)
	  {

		  mp3_decode(&bytesread, 0);
		  buffer_offset = BUFFER_OFFSET_NONE;
	  }

	  if(buffer_offset == BUFFER_OFFSET_FULL)
	  {
		  mp3_decode(&bytesread, 1);
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

void find_track_length(){
	while(1){
		unsigned int bytesread = 0;
		samples += mp3dec_decode_frame(&mp3d, (const uint8_t*) &input_data[0], MP3_BUF, 0, &info);
		memmove(&input_data[0], &input_data[info.frame_bytes], (MP3_BUF - info.frame_bytes));   /**/
		f_read(&FileRead, &input_data[MP3_BUF - info.frame_bytes], info.frame_bytes, &bytesread);  /**/

		if(bytesread == 0){
			unsigned int n = 0;
			while(info.frame_bytes > 0){
				n += info.frame_bytes;
				samples += mp3dec_decode_frame(&mp3d, &input_data[n], MP3_BUF - n, 0, &info);
			}
		}
		if (info.frame_bytes <= 0) break;
	}

}

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
			find_track_length();
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
			mp3_playback(44100);

		}
	}

}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio_codec.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/

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
