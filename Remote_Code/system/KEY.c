#include "stm32f10x.h"                  // Device header
#include "Delay.h"


uint8_t KEY_V;


/*
PB5    EXTI 5     EXTI9_5_IRQn      KEY6
PB8    EXTI 8     EXTI9_5_IRQn      KEY3
PB9    EXTI 9     EXTI9_5_IRQn      KEY4
PB13   EXTI 13    EXTI15_10_IRQn    KEY5
PB14   EXTI 14    EXTI15_10_IRQn    KEY2
PB15   EXTI 15    EXTI15_10_IRQn    KEY1




*/
void KEY_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	
	GPIO_InitTypeDef GPIO_INITSTRCUTUER;
	GPIO_INITSTRCUTUER.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_INITSTRCUTUER.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 |GPIO_Pin_9 |GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ;
	GPIO_INITSTRCUTUER.GPIO_Speed = GPIO_Speed_50MHz ;
	
	GPIO_Init (GPIOB,&GPIO_INITSTRCUTUER);

	
	//EXTI结构体初始化
	EXTI_InitTypeDef EXTI_INITSTRCUTUER;
	
	EXTI_INITSTRCUTUER.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_INITSTRCUTUER.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_INITSTRCUTUER.EXTI_LineCmd = ENABLE;
	
	//PB5
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
	EXTI_INITSTRCUTUER.EXTI_Line = EXTI_Line5;
	
	EXTI_Init (&EXTI_INITSTRCUTUER);
	
	//PB8
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);
	EXTI_INITSTRCUTUER.EXTI_Line = EXTI_Line8;
	
	EXTI_Init (&EXTI_INITSTRCUTUER);	
	
	//PB9
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource9);
	EXTI_INITSTRCUTUER.EXTI_Line = EXTI_Line9;
	
	EXTI_Init (&EXTI_INITSTRCUTUER);	
	
	//PB13
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
	EXTI_INITSTRCUTUER.EXTI_Line = EXTI_Line13;
	
	EXTI_Init (&EXTI_INITSTRCUTUER);	
	
	//PB14
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);
	EXTI_INITSTRCUTUER.EXTI_Line = EXTI_Line14;
	
	EXTI_Init (&EXTI_INITSTRCUTUER);	
	
	//PB15
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource15);
	EXTI_INITSTRCUTUER.EXTI_Line = EXTI_Line15;
	
	EXTI_Init (&EXTI_INITSTRCUTUER);	
	
	
	
	
	//NVIC结构体初始化
	
	NVIC_InitTypeDef NVIC_INITSTRCUTUER;
	
	NVIC_INITSTRCUTUER.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_INITSTRCUTUER.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_INITSTRCUTUER.NVIC_IRQChannelSubPriority = 2;
	NVIC_INITSTRCUTUER.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init (&NVIC_INITSTRCUTUER);
	
	NVIC_INITSTRCUTUER.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_INITSTRCUTUER.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_INITSTRCUTUER.NVIC_IRQChannelSubPriority = 3;
	NVIC_INITSTRCUTUER.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init (&NVIC_INITSTRCUTUER);	
	
	
	
	KEY_V = 0;
}




void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5) == SET)
    {
        // PB5 触发
        KEY_V = 6;
        EXTI_ClearITPendingBit(EXTI_Line5);
    }

    if (EXTI_GetITStatus(EXTI_Line8) == SET)
    {
        // PB8 触发
//		Delay_ms (20);
        KEY_V = 3;
        EXTI_ClearITPendingBit(EXTI_Line8);
    }

    if (EXTI_GetITStatus(EXTI_Line9) == SET)
    {
        // PB9 触发
//		Delay_ms (20);
        KEY_V = 4;
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}



void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line13) == SET)
    {
        // PB13 触发
        KEY_V = 5;
        EXTI_ClearITPendingBit(EXTI_Line13);
    }

    if (EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        // PB14 触发
        KEY_V = 2;
        EXTI_ClearITPendingBit(EXTI_Line14);
    }

    if (EXTI_GetITStatus(EXTI_Line15) == SET)
    {
        // PB15 触发
        KEY_V = 1;
        EXTI_ClearITPendingBit(EXTI_Line15);
    }
}


uint8_t KEY_GET(void)
{
	if (KEY_V !=0)
	{
		Delay_ms(10);
		
		if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 0)KEY_V = 3;
		if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 0)KEY_V = 4;
		
		
		
	}
	
	
	return KEY_V;
	
	

}




