/*
 * mp3player.c
 *
 *  Created on: Oct 8, 2020
 *      Author: Rami
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "ff.h"
#include "string.h"
#include "stm32f4_discovery_audio.h"

/* Private define ------------------------------------------------------------*/

#define MINIMP3_NO_STDIO
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#include "minimp3_ex.h"

/*MP3 BUFFER SIZES*/

#ifdef SEGGER_SYSVIEW_DEBUGGING
#define MP3_BUF             (4 * 1024)
#else
#define MP3_BUF             (8 * 1024)
#endif

#define AUDIO_BUFFER_SIZE	(8 * 1152)

/* Private macro -------------------------------------------------------------*/

#define REMAP(X) (((X - OLD_MIN) * (NEW_MAX - NEW_MIN)) / (OLD_MAX - OLD_MIN)) + NEW_MIN

/* Private variables ---------------------------------------------------------*/

/*FatFs variables*/
static FIL xFileRead;
static DIR xDirectory;

/*minimp3 decoding structures*/
static mp3dec_t xMp3Dec;
static mp3dec_frame_info_t xFrameInfo;

/*encoded mp3 data read from USB storage*/
static char cInputData[MP3_BUF];

/*decoded audio pcm data*/
static uint8_t ucAudioBuffer[AUDIO_BUFFER_SIZE];

/*mp3 play-back variables*/
static __IO uint32_t ulAudioRemSize = 0;
static uint32_t ulMP3DataLength = 0;
static uint64_t ullSamples;
__IO BUFFER_StateTypeDef xBufferOffset = BUFFER_OFFSET_NONE;

/*external flags & variables*/
extern MP3* pxCurrent;
extern uint8_t ucNewSongFlag;
extern uint8_t ucPauseStateFlag;
extern uint8_t ucFindInfoFlag;

static uint32_t ulBytesRead = 0;
static uint32_t ulOldTime = 0;
static uint8_t ucOldPauseStateFlag = 0;

/* Private function prototypes -----------------------------------------------*/
static void prvMp3PlayerFillFirstInputBuffer(uint32_t *pulBytesRead);
static uint8_t prvMp3PlayerIsPaused();
static void prvMp3PlayerPause();
static void prvMp3PlayerResume();
static void prvMp3PlayerDecode(uint32_t *pulBytesRead, int xInputBufPos);
static void prvMp3PlayerClosePlayback();
static size_t prvMp3PlayerMiniIoRead(void* pvBuf, size_t xSize, void* pvUserData);
static int prvMp3PlayerMiniIoSeek(uint64_t ullPosition, void* pvUserData);

/**
  * @brief  fills the input buffer with mp3 data at the beginning of mp3 decoding
  * @param  pulBytesRead : amount of f_read bytes read
  * @retval None
  */
static void prvMp3PlayerFillFirstInputBuffer(uint32_t *pulBytesRead){
    f_lseek(&xFileRead, 0);                                                                                                 /*seek to beginning of file*/
    f_read(&xFileRead, &cInputData[0], MP3_BUF, (void *) pulBytesRead);                                                     /*read mp3 data into input buffer*/
    ulAudioRemSize = ulMP3DataLength - *pulBytesRead;                                                                       /*calculate ulAudioRemSize*/
}

/**
  * @brief  Decodes a single frame of the mp3 file and stores the result in the Audio buffer
  * @param  pulBytesRead : amount of f_read bytes read
  * @param  xInputBufPos  : position to set the Audio Buffer
  * @retval None
  */
static void prvMp3PlayerDecode(uint32_t *pulBytesRead, int xInputBufPos){

    if(!ulAudioRemSize){                                                                                                    /*finish decoding last buffer*/
        static uint32_t n = 0;
        if(xFrameInfo.frame_bytes){                                                                                         /*if there are still more frames left*/
            n += xFrameInfo.frame_bytes;                                                                                        /*track frame position in input buffer*/
                                                                                                                                /*decode input data, store PCM in buffer*/
            ullSamples += mp3dec_decode_frame(&xMp3Dec, (const uint8_t*) &cInputData[n], MP3_BUF - n, (short*) &ucAudioBuffer[xInputBufPos], &xFrameInfo);
        }else{                                                                                                              /*if there are no frames left*/
            ucFindInfoFlag = 1;                                                                                                 /*update mp3 decoding flag*/
            pxCurrent = pxCurrent->pxNext;                                                                                      /*update LL to point to next track*/
        }
    }else{
                                                                                                                            /*decode input data, store PCM in buffer*/
        ullSamples += mp3dec_decode_frame(&xMp3Dec, (const uint8_t*) &cInputData[0], MP3_BUF, (short*) &ucAudioBuffer[xInputBufPos], &xFrameInfo);
        memmove(&cInputData[0], &cInputData[xFrameInfo.frame_bytes], (MP3_BUF - xFrameInfo.frame_bytes));                   /*move input buffer to to correct position*/
        f_read(&xFileRead, &cInputData[MP3_BUF - xFrameInfo.frame_bytes], xFrameInfo.frame_bytes, (void *) pulBytesRead);   /*read more data into end of input buffer*/
        ulAudioRemSize = ulAudioRemSize - *pulBytesRead;                                                                    /*update ulAudioRemSize variable*/
    }
}

/**
  * @brief  Stops Audio playback, closes Mp3 file and sets sample back to 0
  * @param  None
  * @retval None
  */
static void prvMp3PlayerClosePlayback(){
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);                                                                                      /*stop audio play-back*/
    f_close(&xFileRead);                                                                                                    /*close file*/
    ullSamples = 0;
}

/**
  * @brief  Initializes the Mp3 play-back process and fills the first input buffer
  * @param  None
  * @retval None
  */
void vMp3PlayerInit(){
    vUpdateLCDSetCurrentTime(0);                                                                                            /*initialize current mp3 time equal to 0*/

    mp3dec_init(&xMp3Dec);                                                                                                  /*start mininmp3 decoding process*/
    if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, ulUpdateLCDGetSampleRate()) != 0){                                        /* Initialize MP3 player (Codec, DMA, I2C) */
        Error_Handler();                                                                                                        /*error if mp3 player initialization fails*/
    }

    prvMp3PlayerFillFirstInputBuffer(&ulBytesRead);                                                                         /*fill input buffer completely input buffer*/
    prvMp3PlayerDecode(&ulBytesRead, 0);                                                                                    /*decode mp3 data, store result in first half of Audio Buffer*/
    prvMp3PlayerDecode(&ulBytesRead, AUDIO_BUFFER_SIZE/2);                                                                  /*decode mp3 data, store result in second half of Audio Buffer*/

    if(BSP_AUDIO_OUT_Play((uint16_t*)&ucAudioBuffer[0], AUDIO_BUFFER_SIZE) != 0){                                           /*start playing MP3*/
        Error_Handler();
    }

}

/**
  * @brief  Checks whether mp3 play-back is paused
  * @param  None
  * @retval returns 1 if mp3 play-back is paused
  */
static uint8_t prvMp3PlayerIsPaused(){
    if(ucPauseStateFlag == 1 || ucOldPauseStateFlag == 1)
        return 1;
    return 0;
}

/**
  * @brief  Pauses Mp3 Play-back
  * @param  None
  * @retval None
  */
static void prvMp3PlayerPause(){
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);                                                                                      /*stop play-back*/
    ucOldPauseStateFlag = 1;                                                                                                /*set flag ucOldPauseStateFlag*/
}

/**
  * @brief  Resumes Mp3 Play-back
  * @param  None
  * @retval None
  */
static void prvMp3PlayerResume(){
    if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, ulUpdateLCDGetVolume(), ulUpdateLCDGetSampleRate()) != 0){                    /* Initialize MP3 player (Codec, DMA, I2C) */
        Error_Handler();                                                                                                        /*error if mp3 player initialization fails*/
    }
    if (BSP_AUDIO_OUT_Play((uint16_t*)&ucAudioBuffer[0], AUDIO_BUFFER_SIZE) != 0){                                          /*start playing MP3*/
        Error_Handler();
    }
    ucOldPauseStateFlag = 0;                                                                                                /*de-assert ucOldPauseStateFlag*/
}

/**
  * @brief  Manages MP3 decoding process, and fills the Audio buffer when buffer offset flags are set
  * @param  None
  * @retval None
  */
void vMp3PlayerDecodeFrames(){

    while(!prvMp3PlayerIsPaused()){                                                                                         /*enter DMA play-back loop*/
        ulBytesRead = 0;                                                                                                        /*set bytes read back to zero*/

        vUpdateLCDSetCurrentTime(ullSamples / ulUpdateLCDGetSampleRate());                                                      /*update current time*/
        if(ulUpdateLCDGetCurrentTime() != ulOldTime){                                                                           /*if current time does not match old time*/
            ulOldTime = ulUpdateLCDGetCurrentTime();                                                                                /*set old time equal to current time*/
        }

        if(xBufferOffset == BUFFER_OFFSET_HALF){                                                                               /*check if the first half of the Audio Buffer has been transferred*/
            prvMp3PlayerDecode(&ulBytesRead, 0);                                                                                    /*decode next mp3 data to replace it in the Audio buffer*/
            xBufferOffset = BUFFER_OFFSET_NONE;                                                                                     /*update buffer offset*/
        }

        if(xBufferOffset == BUFFER_OFFSET_FULL){                                                                                /*check if the second half of the Audio Buffer has been transferred*/
            prvMp3PlayerDecode(&ulBytesRead, AUDIO_BUFFER_SIZE/2);                                                                  /*decode next mp3 data to replace it in the Audio buffer*/
            xBufferOffset = BUFFER_OFFSET_NONE;                                                                                     /*update buffer offset*/
            break;
        }

        BSP_AUDIO_OUT_SetVolume(ulUpdateLCDGetVolume());                                                                        /*set volume appropriately*/
    }

    if(ucNewSongFlag == 1){                                                                                                     /*if the track changes*/
        ucNewSongFlag = 0;
        if(prvMp3PlayerIsPaused()){                                                                                                 /*if the track is paused*/
            ucPauseStateFlag = 0;                                                                                                   /*resume and then change the track*/
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            prvMp3PlayerResume();
        }
        memset(&ucAudioBuffer[0], 0, AUDIO_BUFFER_SIZE);                                                                            /*fixed bug where audio stutters as track changes*/
        prvMp3PlayerClosePlayback();                                                                                                /*close playback*/
        ucFindInfoFlag = 1;
    }

    if(prvMp3PlayerIsPaused()){                                                                                             /*if Pause Button is pressed or playback is paused*/
        if(ucOldPauseStateFlag == 0){                                                                                           /*if its the first case*/
            prvMp3PlayerPause();                                                                                                  /*pause play-back*/
        }else if(ucPauseStateFlag == 0){                                                                                        /*if button is pressed to resume*/
            prvMp3PlayerResume();                                                                                                 /*Resume play-back*/
        }
    }


}

/**
  * @brief  custom minimp3 io read function
  * @param  f_read buffer
  * @param  data size in bytes
  * @param  mp3 file pointer
  * @retval bytes read
  */
static size_t prvMp3PlayerMiniIoRead(void* pvBuf, size_t xSize, void* pvUserData){
    unsigned int uxBytesRead;
    f_read((FIL*) pvUserData, pvBuf, (unsigned int) xSize, (unsigned int*) &uxBytesRead);
    return (size_t) uxBytesRead;
}

/**
  * @brief  custom minimp3 io seek function
  * @param  seek position
  * @param  mp3 file pointer
  * @retval FRESULT code
  */
static int prvMp3PlayerMiniIoSeek(uint64_t ullPosition, void* pvUserData){
    return f_lseek((FIL*) pvUserData, ullPosition);
}

/**
  * @brief  finds the track length and sample rate of the mp3
  * @param  None
  * @retval None
  */
void vMp3PlayerFindInfo(){

    mp3dec_ex_t xDec;                                                                                                           /*necessary minimp3 variables*/
    mp3dec_io_t xIo;                                                                                                            /* "" "" "" */
    xIo.read = prvMp3PlayerMiniIoRead;                                                                                          /* "" "" "" */
    xIo.seek = prvMp3PlayerMiniIoSeek;                                                                                          /* "" "" "" */
    xIo.read_data = xIo.seek_data = &xFileRead;                                                                                 /* "" "" "" */

    vUpdateLCDSetMp3Track(pxCurrent->pcMp3Name);

    if(f_opendir(&xDirectory, "0:/") == FR_OK){                                                                                 /*Get the read out protection status*/
        if(f_open(&xFileRead, pxCurrent->pcMp3Name , FA_READ) != FR_OK){                                                            /*Open the MP3 file to be played*/
            Error_Handler();                                                                                                        /*error if file does not exist*/
        }
    }else{
        Error_Handler();                                                                                                            /*error if directory can not be opened*/
    }

    ulMP3DataLength = f_size(&xFileRead);                                                                                       /*file length information*/

    if(mp3dec_ex_open_cb(&xDec, &xIo, MP3D_SEEK_TO_SAMPLE)){                                                                    /*find VBR tag*/
        vUpdateLCDSetSampleRate(44100);                                                                                             /*Default behaviour if tag cannot be found*/
        vUpdateLCDSetTotalTime(0);
        return;
    }

    vUpdateLCDSetSampleRate(xDec.info.hz);                                                                                      /*sample rate information*/

    if(xDec.info.channels == 2){                                                                                                /*calculate mp3 track length*/
        vUpdateLCDSetTotalTime(xDec.samples / (xDec.info.hz * 2));                                                              /* "" "" "" */
    }else{                                                                                                                      /* "" "" "" */
        vUpdateLCDSetTotalTime(xDec.samples / (xDec.info.hz));                                                                  /* "" "" "" */
    }
    mp3dec_ex_close(&xDec);                                                                                                     /*free memory allocated by minimp3 library*/
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
    xBufferOffset = BUFFER_OFFSET_HALF;                                                                                         /*set the buffer offset flag*/
}

/**
  * @brief  Calculates the remaining file size and new position of the pointer.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(void){
    xBufferOffset = BUFFER_OFFSET_FULL;                                                                                         /*set the buffer offset flag*/
    BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&ucAudioBuffer[0], AUDIO_BUFFER_SIZE / 2);                                            /*fill buffer with next audio information*/
}

/**
  * @brief  Manages the DMA FIFO error interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(void){
    while (1){                                                                                                                  /*Stop the program with an infinite loop*/
      Error_Handler();                                                                                                          /*Go to error handler*/
    }
}

