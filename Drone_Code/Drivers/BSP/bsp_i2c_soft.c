#include <stm32f4xx.h>
#include "Delay.h"
#include "MAIN.h"


#define SDA_IN_()   (GPIOB->MODER = (GPIOB->MODER & ~(3 << (7*2))) | (0 << (7*2)))
#define SDA_OUT_()  (GPIOB->MODER = (GPIOB->MODER & ~(3 << (7*2))) | (1 << (7*2)))









void IIC_W_SCL(uint8_t V)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_6,(BitAction)V);
	Delay_us(2);
}

void IIC_W_SDA(uint8_t V)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_7,(BitAction)V);
	Delay_us(2);
}

uint8_t IIC_R_SDA(void)
{
	uint8_t V;
	Delay_us(2);
	V = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
	Delay_us(2);
	return V;
}










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
	SDA_OUT_();
	IIC_W_SDA(1);
	IIC_W_SCL(1);
	Delay_us(4);
	IIC_W_SDA(0);
	Delay_us(4);
	IIC_W_SCL(0);
	
}


void My_IIC_Stop(void)
{
	SDA_OUT_();
	IIC_W_SCL(0);
	IIC_W_SDA(0);
	Delay_us(4);
	IIC_W_SCL(1);
	Delay_us(4);
	IIC_W_SDA(1);
	
}

void My_IIC_SendByte(uint8_t Byte)
{
	uint8_t I;
	SDA_OUT_();
	IIC_W_SCL(0);
	for (I=0;I<8;I++)
	{
		IIC_W_SDA( Byte & (0X80>>I));
		IIC_W_SCL(1);
		IIC_W_SCL(0);
	}

}

uint8_t My_IIC_ReceiveByte(void)
{
	uint8_t I , Byte = 0x00;
	SDA_IN_();
//	IIC_W_SDA(1);
	for (I=0;I<8;I++)
	{
		IIC_W_SCL(1);
		if (IIC_R_SDA() ==1)
		{
			Byte |= (0x80>>I);
		}
		IIC_W_SCL(0);
	}

	return Byte;
}



void My_IIC_SendACK(uint8_t ACK)
{
		IIC_W_SCL(0);
		SDA_OUT_();
		IIC_W_SDA( ACK);
		IIC_W_SCL(1);
		IIC_W_SCL(0);


}

uint8_t My_IIC_ReceiveACK(void)
{
	uint8_t  ACK ;
	SDA_IN_();
//	IIC_W_SDA(1);		
//	IIC_W_SDA(1);
	
	IIC_W_SCL(1);
	ACK = IIC_R_SDA();
	IIC_W_SCL(0);
	return ACK;
}





















