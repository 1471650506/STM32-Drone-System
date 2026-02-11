#ifndef __KEY_H
#define __KEY_H



extern uint8_t KEY_V;



void KEY_Init(void);


void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

uint8_t KEY_GET(void);






#endif
