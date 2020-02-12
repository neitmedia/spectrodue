#include "ADS8860.h"
#include "globalvars.h"

static void DIN_Set(void) 
{
	HAL_GPIO_WritePin(DIN_Port, DIN_Pin, GPIO_PIN_SET);
}

static void CONVST_Reset(void) 
{ 
	HAL_GPIO_WritePin(CONVST_Port, CONVST_Pin, GPIO_PIN_RESET);
}

static void CONVST_Set(void) 
{ 
	HAL_GPIO_WritePin(CONVST_Port, CONVST_Pin, GPIO_PIN_SET);
}

uint16_t ADS8860_ReadValue(void)
{
	DIN_Set();
	CONVST_Set();
	delay_600ns();
	CONVST_Reset();
  uint16_t val;
	/*HAL_SPI_Receive(&ADS8860_SPI_Port, &readBuffer, 8, 0xFFFF);
	adcReadBuffer = readBuffer;
	readBuffer = 0;
	HAL_SPI_Receive(&ADS8860_SPI_Port, &readBuffer, 8, 0xFFFF);
	adcReadBuffer = adcReadBuffer << 8;
	adcReadBuffer |= readBuffer;*/
	/*HAL_SPI_Receive(&ADS8860_SPI_Port, buffer, 2, 1);
	adcReadBuffer = ((uint16_t) buffer[0] << 8) | ((uint16_t) buffer[1]);*/
	/*HAL_SPI_Receive(&ADS8860_SPI_Port, &adcReadBuffer, 16, 0xFFFF);*/
	
	HAL_SPI_Receive(&ADS8860_SPI_Port, (uint8_t *)&val, 1, 10);
	
	HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DOUT_Port, DOUT_Pin, GPIO_PIN_RESET);
	return val;
}

void delay_600ns(void)
{
	for (uint8_t i = 0; i < 10; i++)
		__nop();

}

void delay_ticks(int n)
{
	for (uint8_t i = 0; i < n; i++)
		__nop();

}

void ads8860_Init(void)
{
	DIN_Set();
	CONVST_Reset();
}
