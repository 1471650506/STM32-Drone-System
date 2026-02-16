#include <stm32f4xx.h>
#include "Delay.h"
#include "MAIN.h"


//#define SDA_IN_()   (GPIOB->MODER = (GPIOB->MODER & ~(3 << (7*2))) | (0 << (7*2)))
//#define SDA_OUT_()  (GPIOB->MODER = (GPIOB->MODER & ~(3 << (7*2))) | (1 << (7*2)))


#define SCL_H()    (GPIOB-> BSRR = GPIO_Pin_6)
#define SCL_L()    (GPIOB-> BSRR = GPIO_Pin_6 << 16)

#define SDA_H()    (GPIOB-> BSRR = GPIO_Pin_7)
#define SDA_L()    (GPIOB-> BSRR = GPIO_Pin_7 << 16)  


#define READ_SDA() ((GPIOB-> IDR & GPIO_Pin_7 )? 1:0)  



//void IIC_W_SCL(uint8_t V)
//{
//	GPIO_WriteBit(GPIOB,GPIO_Pin_6,(BitAction)V);
//	Delay_us(2);
//}

//void IIC_W_SDA(uint8_t V)
//{
//	GPIO_WriteBit(GPIOB,GPIO_Pin_7,(BitAction)V);
//	Delay_us(2);
//}

//uint8_t IIC_R_SDA(void)
//{
//	uint8_t V;
//	Delay_us(2);
//	V = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
//	Delay_us(2);
//	return V;
//}










void My_IIC_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef gpio_initstrcutuer;
	gpio_initstrcutuer.GPIO_Mode = GPIO_Mode_OUT;
	gpio_initstrcutuer.GPIO_OType = GPIO_OType_OD;
	gpio_initstrcutuer.GPIO_Pin = GPIO_Pin_6;
	gpio_initstrcutuer.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_initstrcutuer.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&gpio_initstrcutuer);
	
	
	

	gpio_initstrcutuer.GPIO_Mode = GPIO_Mode_OUT;
	gpio_initstrcutuer.GPIO_OType = GPIO_OType_OD;
	gpio_initstrcutuer.GPIO_Pin = GPIO_Pin_7;
	gpio_initstrcutuer.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_initstrcutuer.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&gpio_initstrcutuer);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
		
		
//	Delay_ms(10);
}
	
	

void My_IIC_Start(void)
{
	SDA_H();
	SCL_H();
	Delay_us(1);
	SDA_L();
	Delay_us(1);
	SCL_L();
	
}


void My_IIC_Stop(void)
{
	SCL_L();
	SDA_L();
	Delay_us(1);
	SCL_H();
	Delay_us(1);
	SDA_H();
	
}

void My_IIC_SendByte(uint8_t Byte)
{
	uint8_t I;
	SCL_L();
	for (I=0;I<8;I++)
	{
		if (Byte& 0x80)
		{
		SDA_H();
		}
		else
		{
		SDA_L();	
		}
		Byte <<= 1;
		
		
		Delay_us(1);
		SCL_H();
		Delay_us(1);
		SCL_L();
	}

}

uint8_t My_IIC_ReceiveByte(void)
{
	uint8_t I , Byte = 0x00;
	SDA_H();
	for (I=0;I<8;I++)
	{
		SCL_L();
		Delay_us(1);
		
		SCL_H();
		
		Byte <<=1;
		
		if (READ_SDA() ==1)
		{
			Byte ++;
			
		}
		Delay_us(1);
	}
	SCL_L();

	return Byte;
}



void My_IIC_SendACK(uint8_t ACK)
{
		SCL_L();
	
		if (ACK ==1)
		{
			SDA_H();
		}
		else 
		{
			SDA_L();
		}
		Delay_us(1);
		SCL_H();
		Delay_us(1);
		SCL_L();


}

uint8_t My_IIC_ReceiveACK(void)
{
	uint8_t  ACK;
	
	SDA_H();
	Delay_us(1);
	
	SCL_H();
	Delay_us(1);
	
	ACK = READ_SDA();
	
	SCL_L();
	
	return ACK;
}





















