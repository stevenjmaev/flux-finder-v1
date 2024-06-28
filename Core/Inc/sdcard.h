/* vim: set ai et ts=4 sw=4: */
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "stm32f0xx_hal.h"

#define SDCARD_SPI_PORT      hspi1
#define SDCARD_CS_Pin        GPIO_PIN_5 // Arduino shield: D4
#define SDCARD_CS_GPIO_Port  GPIOB

extern SPI_HandleTypeDef SDCARD_SPI_PORT;

// call before initializing any SPI devices
void SDCARD_Unselect();

// all procedures return 0 on success, < 0 on failure

int SDCARD_Init();
int SDCARD_GetBlocksNumber(uint32_t* num);
int SDCARD_ReadSingleBlock(uint32_t blockNum, uint8_t* buff); // sizeof(buff) == 512!
int SDCARD_WriteSingleBlock(uint32_t blockNum, const uint8_t* buff); // sizeof(buff) == 512!

// Read Multiple Blocks
int SDCARD_ReadBegin(uint32_t blockNum);
int SDCARD_ReadData(uint8_t* buff); // sizeof(buff) == 512!
int SDCARD_ReadEnd();

// Write Multiple Blocks
int SDCARD_WriteBegin(uint32_t blockNum);
int SDCARD_WriteData(const uint8_t* buff); // sizeof(buff) == 512!
int SDCARD_WriteEnd();

// TODO: read lock flag? CMD13, SEND_STATUS

#endif // __SDCARD_H__