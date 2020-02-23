/*
The following code is roughly based on code taken from
https://github.com/Floyd-Fish/ADS8860-STM32F4
*/

#include "main.h"

// Declaration of DIN PIN
#define DIN_Pin GPIO_PIN_7
#define DIN_Port GPIOA

// Declaration of CONVST PIN
#define CONVST_Pin GPIO_PIN_4
#define CONVST_Port GPIOA

// Declaration of used SPI Port
#define ADS8860_SPI_Port hspi1
extern SPI_HandleTypeDef ADS8860_SPI_Port;

// Declaration of external used functions
uint16_t ADS8860_ReadValue(void);
void ads8860_Init(void);
