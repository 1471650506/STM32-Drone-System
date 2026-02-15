#ifndef __My_USART1_H
#define __My_USART1_H

extern  float usart_s [32];




void VOFA_Send3(float ch1, float ch2, float ch3);
void USART1_Init(uint32_t baud);
//void VOFA_SendEuler(EulerAngles_t *e);
void USART1_SendByte(uint8_t data);
void VOFA_SendFrameEnd();



#endif 