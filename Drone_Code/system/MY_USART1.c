#include "stm32f4xx.h"
#include "imu.h"

float usart_s [32] ;


void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    /* 开启时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* PA9 TX, PA10 RX 复用功能 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART 配置 */
    USART_InitStruct.USART_BaudRate = baud;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    USART_Cmd(USART1, ENABLE);
}

/* 发送一个字节 */
void USART1_SendByte(uint8_t data)
{
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = data;
}






void VOFA_SendFloat(float f)
{
    uint8_t *p = (uint8_t *)&f;
    USART1_SendByte(p[0]);
    USART1_SendByte(p[1]);
    USART1_SendByte(p[2]);
    USART1_SendByte(p[3]);
}

void VOFA_SendFrameEnd()
{
    USART1_SendByte(0x00);
    USART1_SendByte(0x00);
    USART1_SendByte(0x80);
    USART1_SendByte(0x7F);
}


void VOFA_Send3(float ch1, float ch2, float ch3)
{
    VOFA_SendFloat(ch1);
    VOFA_SendFloat(ch2);
    VOFA_SendFloat(ch3);

    VOFA_SendFrameEnd();
}

//void VOFA_SendEuler(EulerAngles_t *e)
//{
//    VOFA_SendFloat(e->roll);
//    VOFA_SendFloat(e->pitch);
//    VOFA_SendFloat(e->yaw);

//    VOFA_SendFrameEnd();
//}


