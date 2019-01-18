#include "msp.h"

void SPI_Send_Command(void);

void SPI_Send_Data(uint8_t data);

uint8_t SPI_Read_Data(void);

void Encoder_Init(void);

int32_t Encoder_Read(int16_t encoder);

void Encoder_Count_Clear(void);

