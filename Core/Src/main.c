/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define REMAP(X) (((X - OLD_MIN) * (NEW_MAX - NEW_MIN)) / (OLD_MAX - OLD_MIN)) + NEW_MIN

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TaskHandle_t xMP3TaskHandle = NULL, xUpdateLCDTaskHandle = NULL,xADCTaskHandle = NULL, xButtonsTaskHandle = NULL;
MP3 *pxStart = NULL;
MP3 *pxCurrent = NULL;
uint8_t ucNewSongFlag = 1;
uint8_t ucFindInfoFlag = 1;
uint8_t ucPauseStateFlag = 0;
extern ApplicationTypeDef Appli_state;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

void vMP3Playback_TaskHandler(void *params);
void vUpdateLCD_TaskHandler(void *params);
void vReadADC_TaskHandler(void *params);
void vReadInputButtons_TaskHandler(void *params);

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  gets the file extension
  * @retval char* to first character of file extension
  */
const char* pcGetExtension(const char *pcFile){
    const char *pcPeriod = strrchr(pcFile, '.');                                                                                    /*pointer to last occurrence of "."*/
    if(!pcPeriod || pcPeriod == pcFile) return "";                                                                                  /*return "" if no "."*/
    return pcPeriod + 1;                                                                                                            /* return pointer to char after "."*/
}

/**
  * @brief  build a linked list of mp3 files on usb drive
  * @retval none
  */
void vBuildMp3List(){
    DIR xDirectory;                                                                                                                 /*directory structure*/
    FILINFO xFinf;                                                                                                                  /*file info structure*/

    if(f_opendir(&xDirectory, "0:/") == FR_OK){                                                                                     /*get the read out protection status*/
        while(f_readdir(&xDirectory, &xFinf) == FR_OK){                                                                             /*start reading directory entries*/

            if(xFinf.fname[0] == 0)                                                                                                 /*exit loop when finished*/
                break;                                                                                                              /* "" "" "" */

            if(strcmp("mp3", pcGetExtension((char*) xFinf.fname)) == 0){                                                            /*make sure file extension is .mp3*/
                vSongLLAddEnd(&pxStart, pxSongLLNewElement((char*) xFinf.fname));                                                       /*add file to linked list of mp3s*/
            }
        }
        vSongLLCircularizeList(pxStart);                                                                                            /*head and tail of LL point to each other*/
        pxCurrent = pxStart;                                                                                                        /*current music track is at head of LL*/
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /*SEGGER SYSVIEW RECORDING*/
#ifdef SEGGER_SYSVIEW_DEBUGGING
    DWT->CTRL |= (1 << 0);

    NVIC_SetPriorityGrouping( 0 );

    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();
#endif

    /* MCU Configuration--------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FATFS_Init();
    MX_ADC1_Init();

    LCM1602a_init(TWO_LINE_DISPLAY);

    /* Create the thread(s) */
    xTaskCreate(vMP3Playback_TaskHandler,      "Mp3 Play-back", 4500,                     NULL, 2, &xMP3TaskHandle);
    xTaskCreate(vUpdateLCD_TaskHandler,        "Update LCD",    250,                      NULL, 2, &xUpdateLCDTaskHandle);
    xTaskCreate(vReadADC_TaskHandler,          "Read ADC",      configMINIMAL_STACK_SIZE, NULL, 2, &xADCTaskHandle);
    xTaskCreate(vReadInputButtons_TaskHandler, "Read Buttons",  configMINIMAL_STACK_SIZE, NULL, 2, &xButtonsTaskHandle);

    /* Start scheduler */
    vTaskStartScheduler();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    while (1)
    {
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin PB12 PB14 */
  GPIO_InitStruct.Pin = BOOT1_Pin|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : lcm1602a data pins*/
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
  					   |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : lcm1602a control pins*/
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*set lcm1602a Data Ports in driver file*/
  uint16_t data_pins[8] = {GPIO_PIN_7, GPIO_PIN_9, GPIO_PIN_11, GPIO_PIN_13, GPIO_PIN_8, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_14};
  uint16_t control_pins[3] = {GPIO_PIN_1, GPIO_PIN_5, GPIO_PIN_4};

  LCM1602a_Set_DATA8(GPIOE, data_pins, GPIOC, control_pins);
}

/**
  * @brief  Task: Handles USB processing and Mp3 Decoding and play-back
  * @param  params: Not used
  * @retval None
  */
void vMP3Playback_TaskHandler(void *params)
{
    MX_USB_HOST_Init();                                                                                                            /*init code for USB_HOST*/
    static uint8_t ucDriveMountedFlag = 0;                                                                                         /*Drive Mounted Flag*/

    for(;;){                                                                                                                       /*Infinite loop*/
        if(Appli_state == APPLICATION_READY && !ucDriveMountedFlag){                                                                   /*if Ready and Drive is not Mounted*/
            ucDriveMountedFlag = 1;                                                                                                    /*Set the Drive Mounted Flag*/
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
            if(f_mount(&USBHFatFS, (const TCHAR*)USBHPath, 0) == FR_OK){                                                               /*Mount USB drive*/
                vBuildMp3List();                                                                                                       /*Build Mp3 LL*/
            }
        }

        if(ucFindInfoFlag && ucDriveMountedFlag){                                                                                  /*if New Song Flag Set and Drive Mounted Set*/
            ucFindInfoFlag = 0;                                                                                                        /*Un-Set New Song Flag*/
            vMp3PlayerFindInfo();                                                                                                      /*Find Mp3 Track info*/
            vMp3PlayerInit();                                                                                                          /*init Mp3 Playback*/
        }

        for(;;){
            if(!ucFindInfoFlag && ucDriveMountedFlag){                                                                             /*if New Song Flag Not Set and Drive Mounted Set*/
                vMp3PlayerDecodeFrames();                                                                                              /*Decode and Play a couple Mp3 Frasmes*/
                taskYIELD();                                                                                                           /*Yield Task*/
            }else{
                break;
            }
        }
    }
}

/**
  * @brief  Task: Updates LCD Screen
  * @param  params: Not used
  * @retval None
  */
void vUpdateLCD_TaskHandler(void *params){
    for(;;){
        vUpdateLCDScreen();                                                                                                        /*Update LCD Screen With New Information*/
        vTaskDelay(500);                                                                                                           /*Block Task for 500 ms*/                                                                                                          /*Yield Task*/
    }
}

/**
  * @brief  Poll ADC
  * @param  params: Not used
  * @retval None
  */
void vReadADC_TaskHandler(void *params){
    for(;;){
        HAL_ADC_Start(&hadc1);                                                                                                     /*start ADC conversion*/
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);                                                                          /*poll ADC*/
        vUpdateLCDSetVolume(REMAP(HAL_ADC_GetValue(&hadc1)));                                                                      /*get the ADC value*/
        vTaskDelay(500);                                                                                                           /*Block Task for 500 ms*/
    }
}

/**
  * @brief  Poll Buttons
  * @param  params: Not used
  * @retval None
  */
void vReadInputButtons_TaskHandler(void *params){
    /* Infinite loop */
    for(;;){
        if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == 0){                                                                             /*if Prev button is pressed*/
            ucNewSongFlag = 1;                                                                                                         /*Set the New Song Flag*/
            pxCurrent = pxCurrent->pxPrev;                                                                                             /*Set LL Previous Node*/
            vTaskDelay(400);                                                                                                           /*Block Task Button Debouncing*/

        }else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0){                                                                       /*if Pause button is pressed*/
            ucPauseStateFlag = ucPauseStateFlag ^ 1;                                                                                   /*invert Pause State Flag*/
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            vTaskDelay(400);                                                                                                           /*Block Task Button Debouncing*/

        }else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0){                                                                       /*if Next button is pressed*/
            ucNewSongFlag = 1;                                                                                                         /*Set the New Song Flag*/
            pxCurrent = pxCurrent->pxNext;                                                                                             /*Set LL to Next Node*/
            vTaskDelay(400);                                                                                                           /*Block Task Button Debouncing*/
        }
        vTaskDelay(50);                                                                                                            /*Block Task for 50 ms*/
    }
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
