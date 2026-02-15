#include <stm32f4xx.h>



void spi_csn(uint8_t data)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,(BitAction)data);
	
}




void My_spi_init()
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);  // SCK
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);  // MISO
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);  // MOSI

	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    
    // csn = PB12
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_SetBits(GPIOB, GPIO_Pin_12);   // NSS 高电平



	SPI_InitTypeDef  SPI_InitStruct;
    SPI_I2S_DeInit(SPI2);
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;  
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;   // 模式0
    SPI_InitStruct.SPI_NSS  = SPI_NSS_Soft;     
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;

    SPI_Init(SPI2, &SPI_InitStruct);

    SPI_Cmd(SPI2, ENABLE);
}



uint8_t My_spi_ReadWrite(uint8_t Data)
{
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)!=SET);
    SPI_I2S_SendData(SPI2,  Data);

    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)!=SET);
    return SPI_I2S_ReceiveData(SPI2);
}







