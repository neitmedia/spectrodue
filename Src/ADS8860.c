/*
The following code is roughly based on code taken from
https://github.com/Floyd-Fish/ADS8860-STM32F4
*/

#include "ADS8860.h"

static void DIN_Set(void) 
{
	// Set DIN Pin to HIGH level
	HAL_GPIO_WritePin(DIN_Port, DIN_Pin, GPIO_PIN_SET);
}

static void CONVST_Reset(void) 
{ 
	// Set CONVST Pin to LOW level
	HAL_GPIO_WritePin(CONVST_Port, CONVST_Pin, GPIO_PIN_RESET);
}

static void CONVST_Set(void) 
{ 
	// Set CONVST Pin to HIGH level
	HAL_GPIO_WritePin(CONVST_Port, CONVST_Pin, GPIO_PIN_SET);
}

static void CONVST_Delay(void)
{
	// Do nothing for some cycles
	for (uint8_t i = 0; i < 3; i++) {
		__nop();
	}
}

/* The following function gives out a
short pulse (~ 250 ns) on CONVST that tells the ADC to start sampling.
After this, it reads in 16 bits via SPI */
uint16_t ADS8860_ReadValue(void)
{
	CONVST_Set();
	CONVST_Delay();
	CONVST_Reset();
  
	uint16_t val;
	HAL_SPI_Receive(&ADS8860_SPI_Port, (uint8_t *)&val, 1, 10);
	
	return val;
}

/* The following function sets DIN to HIGH level and CONVST to LOW level */
void ads8860_Init(void)
{
	DIN_Set();
	CONVST_Reset();
}
