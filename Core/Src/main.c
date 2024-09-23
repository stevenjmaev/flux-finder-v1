/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "matrix.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile uint32_t g_ms_count = 0;
volatile uint32_t g_btn0_time = 0;
volatile uint32_t g_btn1_time = 0;
volatile uint32_t g_btn2_time = 0;
volatile uint8_t g_btn0_state = 0;
volatile uint8_t g_btn1_state = 0;
volatile uint8_t g_btn2_state = 0;

static int8_t btn_counter = 0;

volatile unsigned int hall_readings [NUM_PX] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

  void UART_Printf(const char* fmt, ...) {
      char buff[256];
      va_list args;
      va_start(args, fmt);
      vsnprintf(buff, sizeof(buff), fmt, args);
      va_end(args);
      HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
  }


  void ff_init() {
      FATFS fs;
      FRESULT res;
      UART_Printf("Ready!\r\n");

      // mount the default drive
      res = f_mount(&fs, "", 0);
      if(res != FR_OK) {
          UART_Printf("f_mount() failed, res = %d\r\n", res);
          return;
      }

      UART_Printf("f_mount() done!\r\n");

      uint32_t freeClust;
      FATFS* fs_ptr = &fs;
      res = f_getfree("", &freeClust, &fs_ptr); // Warning! This fills fs.n_fatent and fs.csize!
      if(res != FR_OK) {
          UART_Printf("f_getfree() failed, res = %d\r\n", res);
          return;
      }

      UART_Printf("f_getfree() done!\r\n");

      uint32_t totalBlocks = (fs.n_fatent - 2) * fs.csize;
      uint32_t freeBlocks = freeClust * fs.csize;

      UART_Printf("Total blocks: %lu (%lu Mb)\r\n", totalBlocks, totalBlocks / 2000);
      UART_Printf("Free blocks: %lu (%lu Mb)\r\n", freeBlocks, freeBlocks / 2000);

      DIR dir;
      res = f_opendir(&dir, "/");
      if(res != FR_OK) {
          UART_Printf("f_opendir() failed, res = %d\r\n", res);
          return;
      }

      FILINFO fileInfo;
      uint32_t totalFiles = 0;
      uint32_t totalDirs = 0;
      UART_Printf("--------\r\nRoot directory:\r\n");
      for(;;) {
          res = f_readdir(&dir, &fileInfo);
          if((res != FR_OK) || (fileInfo.fname[0] == '\0')) {
              break;
          }
          
          if(fileInfo.fattrib & AM_DIR) {
              UART_Printf("  DIR  %s\r\n", fileInfo.fname);
              totalDirs++;
          } else {
              UART_Printf("  FILE %s\r\n", fileInfo.fname);
              totalFiles++;
          }
      }

      UART_Printf("(total: %lu dirs, %lu files)\r\n--------\r\n", totalDirs, totalFiles);

      res = f_closedir(&dir);
      if(res != FR_OK) {
          UART_Printf("f_closedir() failed, res = %d\r\n", res);
          return;
      }

      UART_Printf("Writing to log.txt...\r\n");

      char writeBuff[128];
      snprintf(writeBuff, sizeof(writeBuff), "Total blocks: %lu (%lu Mb); Free blocks: %lu (%lu Mb)\r\n",
          totalBlocks, totalBlocks / 2000,
          freeBlocks, freeBlocks / 2000);

      FIL logFile;
      res = f_open(&logFile, "log.txt", FA_OPEN_ALWAYS | FA_WRITE);
      if(res != FR_OK) {
          UART_Printf("f_open() failed, res = %d\r\n", res);
          return;
      }

      unsigned int bytesToWrite = strlen(writeBuff);
      unsigned int bytesWritten;
      res = f_write(&logFile, writeBuff, bytesToWrite, &bytesWritten);
      if(res != FR_OK) {
          UART_Printf("f_write() failed, res = %d\r\n", res);
          return;
      }

      if(bytesWritten < bytesToWrite) {
          UART_Printf("WARNING! Disk is full, bytesToWrite = %lu, bytesWritten = %lu\r\n", bytesToWrite, bytesWritten);
      }

      res = f_close(&logFile);
      if(res != FR_OK) {
          UART_Printf("f_close() failed, res = %d\r\n", res);
          return;
      }

      UART_Printf("Reading file...\r\n");
      FIL msgFile;
      res = f_open(&msgFile, "log.txt", FA_READ);
      if(res != FR_OK) {
          UART_Printf("f_open() failed, res = %d\r\n", res);
          return;
      }

      char readBuff[128];
      unsigned int bytesRead;
      res = f_read(&msgFile, readBuff, sizeof(readBuff)-1, &bytesRead);
      if(res != FR_OK) {
          UART_Printf("f_read() failed, res = %d\r\n", res);
          return;
      }

      readBuff[bytesRead] = '\0';
      UART_Printf("```\r\n%s\r\n```\r\n", readBuff);

      res = f_close(&msgFile);
      if(res != FR_OK) {
          UART_Printf("f_close() failed, res = %d\r\n", res);
          return;
      }

      // Unmount
      res = f_mount(NULL, "", 0);
      if(res != FR_OK) {
          UART_Printf("Unmount failed, res = %d\r\n", res);
          return;
      }

      UART_Printf("Done!\r\n");
  }


int test_fs(void){
   UART_Printf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(1000); //a short delay is important to let the SD card settle

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
    UART_Printf("f_mount error (%i)\r\n", fres);
    while(1);
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
    UART_Printf("f_getfree error (%i)\r\n", fres);
    while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  UART_Printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  //Now let's try to open file "test.txt"
  fres = f_open(&fil, "test.txt", FA_READ);
  if (fres != FR_OK) {
    UART_Printf("f_open error (%i)\r\n");
    while(1);
  }
  UART_Printf("I was able to open 'test.txt' for reading!\r\n");

  //Read 30 bytes from "test.txt" on the SD card
  BYTE readBuf[30];

  //We can either use f_read OR f_gets to get data out of files
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if(rres != 0) {
    UART_Printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  } else {
    UART_Printf("f_gets error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //Now let's try and write a file "write.txt"
  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
    UART_Printf("I was able to open 'write.txt' for writing\r\n");
  } else {
    UART_Printf("f_open error (%i)\r\n", fres);
  }

  //Copy in a string
  strncpy((char*)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if(fres == FR_OK) {
    UART_Printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  } else {
    UART_Printf("f_write error (%i)\r\n");
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);

  return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  uint8_t tmp;
  if (GPIO_Pin == BTN0_Pin){
    tmp = HAL_GPIO_ReadPin(BTN0_GPIO_Port, BTN0_Pin);
    if (tmp != g_btn0_state) {
      g_btn0_time = g_ms_count;
      g_btn0_state = tmp;
    }
  }
  else if (GPIO_Pin == BTN1_Pin){
    tmp = HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);
    if (tmp != g_btn1_state) {
      g_btn1_time = g_ms_count;
      g_btn1_state = tmp;
    }
  }
  else if (GPIO_Pin == BTN2_Pin){
    tmp = HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin);
    if (tmp != g_btn2_state) {
      g_btn2_time = g_ms_count;
      g_btn2_state = tmp;
    }
  }
}

void handle_buttons(void){
  uint32_t time;
  time = g_ms_count;
  if (g_btn0_state && (time > g_btn0_time + DEBOUNCE_TIME_MS)){
    // btn0 is UP
    btn_counter++;
    update_frame_brightness(1);
    g_btn0_state = 0; // we've already handled that button press, so consider it as unpressed
  }
  if (g_btn1_state && (time > g_btn1_time + DEBOUNCE_TIME_MS)){
    // btn1 is DOWN
    btn_counter--;
    update_frame_brightness(0);
    g_btn1_state = 0; // we've already handled that button press, so consider it as unpressed
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int i = 0;
  int j = 0;
  int len = 0;
  char buf [64] = {'\0'};

  snprintf(buf, sizeof(buf), "hello world!\n\r");
  HAL_UART_Transmit(&huart1, buf, sizeof(buf), HAL_MAX_DELAY);
  ff_init();
  // test_fs();

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
  
  init_test_frame();

  htim3.Instance->CCR4 = 1;
  HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t*)&dma_buf, DMA_LEN);

  HAL_TIM_Base_Start_IT(&htim6);

  matrix_select_idx(0);
  HAL_ADC_Start_IT(&hadc);
  
  uint32_t prev_ts = 0;
  while (1)
  {
    handle_buttons();
    
    if (g_ms_count > prev_ts + 500) {
      prev_ts = g_ms_count;
      i = (i + 1) % 3;

      // switch(i){
      // case 0:
      //   HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
      //   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
      //   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
      //   break;
      // case 1:
      //   HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
      //   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
      //   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
      //   break;
      // case 2:
      //   HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
      //   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
      //   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
      //   break;
      // }
      
      // #define ENABLE_UART
      #ifdef ENABLE_UART
        len = snprintf(buf, sizeof(buf), "  frame_pxs[0] = 0x%06x\n\r", frame_pxs[0]);
        HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
        len = snprintf(buf, sizeof(buf), "  btn_counter = %d\n\r", btn_counter);
        HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
      for (j = 0; j < NUM_PX; j++){
        len = snprintf(buf, sizeof(buf), "hall_readings[%d]=%d\n\r", j, hall_readings[0]);
        HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
      }
      #endif
      }

      for (j = 0; j < NUM_PX; j++){
        int32_t diff;
        diff = hall_readings[j] - 1550;
        if (ABS(diff) <= 50)        set_px_color(j, 0x001100);
        else if (ABS(diff) <= 52)  set_px_color(j, 0x020E00);
        else if (ABS(diff) <= 55)  set_px_color(j, 0x040C00);
        else if (ABS(diff) <= 58)  set_px_color(j, 0x060B00);
        else if (ABS(diff) <= 60)  set_px_color(j, 0x081000);
        else if (ABS(diff) <= 67)  set_px_color(j, 0x0A0900);
        else if (ABS(diff) <= 75)  set_px_color(j, 0x0C0800);
        else if (ABS(diff) <= 90)  set_px_color(j, 0x0D0800);
        else if (ABS(diff) <= 100)  set_px_color(j, 0x0E0600);
        else if (ABS(diff) <= 150)  set_px_color(j, 0x0F0400);
        else if (ABS(diff) <= 200)  set_px_color(j, 0x100200);
        else                        set_px_color(j, 0x110000);
      }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_2;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_4;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_5;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_6;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_7;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_8;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  // NOTE: Cannot reasonably generate us precision running timer: http://www.efton.sk/STM32/gotcha/g15.html
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 48;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 48;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|ROW0_Pin|COL1_Pin|COL2_Pin
                          |COL3_Pin|LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW2_Pin|ROW3_Pin|COL0_Pin|SPI1_CS0_Pin
                          |LED1_Pin|LED2_Pin|ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN0_Pin BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN0_Pin|BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 ROW0_Pin COL1_Pin COL2_Pin
                           COL3_Pin LED0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|ROW0_Pin|COL1_Pin|COL2_Pin
                          |COL3_Pin|LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW2_Pin ROW3_Pin COL0_Pin LED1_Pin
                           LED2_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = ROW2_Pin|ROW3_Pin|COL0_Pin|LED1_Pin
                          |LED2_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS0_Pin */
  GPIO_InitStruct.Pin = SPI1_CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_DETECT_Pin */
  GPIO_InitStruct.Pin = uSD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
