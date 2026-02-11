#include "stm32f4xx.h"
#include "Delay.h"       

/* 使用 PB6 → SCL
 *     PB7 → SDA
 */

#define I2C_PORT GPIOB
#define I2C_SCL  GPIO_Pin_6
#define I2C_SDA  GPIO_Pin_7


/* 宏：SCL, SDA 输出高低 */
#define SCL_H() GPIO_SetBits(I2C_PORT, I2C_SCL)
#define SCL_L() GPIO_ResetBits(I2C_PORT, I2C_SCL)

#define SDA_H() GPIO_SetBits(I2C_PORT, I2C_SDA)
#define SDA_L() GPIO_ResetBits(I2C_PORT, I2C_SDA)

/* 切换 SDA 为输入/输出 */
#define SDA_IN()  {GPIOB->MODER&= ~(3<<14);GPIOB->MODER |=(0<<14);}
#define SDA_OUT()  {GPIOB->MODER&= ~(3<<14);GPIOB->MODER |=(1<<14);}


/* 读 SDA */
#define SDA_READ() GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA)



/* 初始化 */
void Soft_I2C_Init(void)
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
}


/* Start 条件：SCL高时 SDA 从高 → 低 */
void Soft_I2C_Start(void)
{
    SDA_OUT();
    SDA_H();
    SCL_H();
    Delay_us(5);
    SDA_L();
    Delay_us(5);
    SCL_L();
}


/* Stop 条件：SCL高时 SDA 从低 → 高 */
void Soft_I2C_Stop(void)
{
    SDA_OUT();
    SCL_L();
    SDA_L();
    Delay_us(5);
    SCL_H();
    Delay_us(5);
    SDA_H();
    Delay_us(5);
}


/* 写一个字节，返回从机是否ACK （0=ACK，1=NACK） */
uint8_t Soft_I2C_WriteByte(uint8_t data)
{
    SDA_OUT();
    for (uint8_t i = 0; i < 8; i++)
    {
        SCL_L();
        if (data & 0x80)
            SDA_H();
        else
            SDA_L();

        Delay_us(3);
        SCL_H();
        Delay_us(5);
        SCL_L();

        data <<= 1;
    }

    /* 读取 ACK */
    SDA_IN();
    SCL_H();
    Delay_us(5);

    uint8_t ack = SDA_READ();
    SCL_L();

    return ack;     // 0 为成功
}


/* 读一个字节，ack=1 发送 ACK，ack=0 发送 NACK */
uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;

    SDA_IN();

    for (uint8_t i = 0; i < 8; i++)
    {
        SCL_H();
        Delay_us(5);

        data <<= 1;
        if (SDA_READ()) data |= 1;

        SCL_L();
        Delay_us(5);
    }

    /* 发 ACK/NACK */
    SDA_OUT();
    if (ack) SDA_L();   // ACK = 0电平
    else     SDA_H();   // NACK = 1电平

    SCL_H();
    Delay_us(5);
    SCL_L();
    Delay_us(5);

    SDA_IN(); // 释放 SDA

    return data;
}
