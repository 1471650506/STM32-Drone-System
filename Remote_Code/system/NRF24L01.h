#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "NRF24L01_Define.h" 




extern uint8_t NRF24L01_SendByte[32];
extern uint8_t NRF24L01_ReceiveByte[32];







void NRF24L01_W_CE(uint8_t Val);

void NRF24L01_W_CSN(uint8_t Val);

//void NRF24L01_W_SCK(uint8_t Val);

//void NRF24L01_W_MOSI(uint8_t Val);

//uint8_t NRF24L01_R_MISO(void);

void NRF24L01_GPIO_Init(void);

uint8_t NRF24L01_SPI_Swap(uint8_t Byte);



void NRF24L01_WriteRge(uint8_t Reg,uint8_t Data);

uint8_t NRF24L01_ReadReg(uint8_t Reg);

void NRF24L01_WriteRgeS(uint8_t Reg,uint8_t* DataArray,uint8_t count);

void NRF24L01_ReadRegS(uint8_t Reg,uint8_t* DataArray,uint8_t count);






void NRF24L01_Init();


uint8_t NRF24L01_Send(void);


uint8_t NRF24L01_Receive(void);


uint8_t NRF24L01_Receive_Link(void);



void NRF24L01_RxMode(void);


void NRF24L01_TxMode(void);



#endif
