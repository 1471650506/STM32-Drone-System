#ifndef __My_IIC_H
#define __My_IIC_H



void My_IIC_Init();

void My_IIC_Start(void);

void My_IIC_Stop(void);

void My_IIC_SendByte(uint8_t Byte);

uint8_t My_IIC_ReceiveByte(void);

void My_IIC_SendACK(uint8_t ACK);

uint8_t My_IIC_ReceiveACK(void);





//void IIC_Init(void);
//void IIC_Start(void);
//void IIC_Stop(void);
//void IIC_Send_Byte(u8 txd);
//u8 IIC_Read_Byte(unsigned char ack);
//u8 IIC_Wait_Ack(void);
//uint8_t My_IIC_ReceiveACK(void);


#endif 

