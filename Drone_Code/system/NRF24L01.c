#include <stm32f4xx.h>
#include "NRF24L01.h" 
#include "Delay.h"
#include "My_spi.h"


uint8_t RxAddress[5]={0X22,0X22,0X22,0X22,0X22};
uint8_t TxAddress[5]={0X22,0X22,0X22,0X22,0X22};

uint8_t NRF24L01_SendByte[32];
uint8_t NRF24L01_ReceiveByte[32];

#define NRF24L01_RX_SendByte   32

/*引脚定义    PA3---PA7*/


uint32_t timeout = 1000;


void NRF24L01_W_CE(uint8_t Val)
{
	
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)Val);

}











void NRF24L01_GPIO_Init(void)
{

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // CE = PB10
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    NRF24L01_W_CE(0);

}




void NRF24L01_WriteRge(uint8_t Reg,uint8_t Data)
{
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_W_REGISTER | Reg);
	 My_spi_ReadWrite(Data);
	 spi_csn(1);

}


uint8_t NRF24L01_ReadReg(uint8_t Reg)
{
	 uint8_t Data;
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_R_REGISTER | Reg);
	 Data = My_spi_ReadWrite(NRF24L01_NOP);
	 spi_csn(1);	 
	 return Data;

}

void NRF24L01_WriteRgeS(uint8_t Reg,uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_W_REGISTER | Reg);
	for (i=0;i<count;i++)
	{
		 My_spi_ReadWrite(DataArray[i]);
	}
	 spi_csn(1);

}


void NRF24L01_ReadRegS(uint8_t Reg,uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_R_REGISTER | Reg);
	 for (i=0;i<count;i++)
     {
		DataArray[i] = My_spi_ReadWrite(NRF24L01_NOP);
	 }
	 spi_csn(1);	 

}


void NRF24L01_WriteTxFIFO(uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_W_TX_PAYLOAD);
	for (i=0;i<count;i++)
	{
		 My_spi_ReadWrite(DataArray[i]);		
		
	}
	 spi_csn(1);

}


void NRF24L01_ReadRxFIFO(uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 uint16_t temp;
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_R_RX_PAYLOAD);
	 for (i=0;i<count;i++)
     {
		DataArray[i] = My_spi_ReadWrite(NRF24L01_NOP);
		 
		 
	 }
	 spi_csn(1);	 

}

void NRF24L01_ClearRxFIFO(void)
{
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_FLUSH_RX);
	 spi_csn(1);	 

}


void NRF24L01_ClearTxFIFO(void)
{
	 spi_csn(0);
	 My_spi_ReadWrite(NRF24L01_FLUSH_TX);
	 spi_csn(1);	 

}


uint8_t NRF24L01_ReadStatus(void)
{
	 uint8_t Status;
	 spi_csn(0);
	 Status = My_spi_ReadWrite(NRF24L01_NOP);
	 spi_csn(1);	 
     return Status;
}


void NRF24L01_PowerDown(void)
{
	 uint8_t Config;
	
	 NRF24L01_W_CE(0);
	
	 Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	 Config &= ~0X02;
	 NRF24L01_WriteRge(NRF24L01_CONFIG,Config);

}

void NRF24L01_StandbyI(void)
{
	 uint8_t Config;
	
	 NRF24L01_W_CE(0);
	
	 Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	 Config |= 0X02;
	 NRF24L01_WriteRge(NRF24L01_CONFIG,Config);

}

void NRF24L01_RxMode(void)
{
	 uint8_t Config;
	
	 NRF24L01_W_CE(0);
	
	 Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	 Config |= 0X03;
	 NRF24L01_WriteRge(NRF24L01_CONFIG,Config);
	
	
	Delay_ms (2);
	 NRF24L01_W_CE(1);
	
	
}

void NRF24L01_TxMode(void)
{
	 uint8_t Config;
	
	 NRF24L01_W_CE(0);
	
	 Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	 Config |= 0X02;
	 Config &= ~0X01;
	
	 NRF24L01_WriteRge(NRF24L01_CONFIG,Config);
	
	Delay_ms(2);
	
	 NRF24L01_W_CE(1);
}


void NRF24L01_Init_()
{
	My_spi_init();
	NRF24L01_GPIO_Init();
	
	
	NRF24L01_W_CE(0);
	
	NRF24L01_WriteRge(NRF24L01_CONFIG,0X0F);
	NRF24L01_WriteRge(NRF24L01_EN_AA,0X01);
	NRF24L01_WriteRge(NRF24L01_EN_RXADDR,0X01);
	NRF24L01_WriteRge(NRF24L01_SETUP_AW,0X03);
	NRF24L01_WriteRge(NRF24L01_SETUP_RETR,0X03);
	NRF24L01_WriteRge(NRF24L01_RF_CH,0X90);
	NRF24L01_WriteRge(NRF24L01_RF_SETUP,0X0E);
	
	
	NRF24L01_WriteRgeS(NRF24L01_RX_ADDR_P0,RxAddress,5);
	
	
	NRF24L01_WriteRge(NRF24L01_RX_PW_P0,NRF24L01_RX_SendByte);
	
	
	NRF24L01_ClearRxFIFO();
	
	
//	NRF24L01_RxMode();
	
	
	
	
	
}


uint8_t NRF24L01_Send(void)
{
//	uint32_t COUNT = 500000;
	uint8_t CONFIG;
	uint8_t Status;
	NRF24L01_WriteRgeS(NRF24L01_TX_ADDR,TxAddress,5);
	NRF24L01_WriteTxFIFO(NRF24L01_SendByte,NRF24L01_RX_SendByte);
	NRF24L01_WriteRgeS(NRF24L01_RX_ADDR_P0,TxAddress,5);
	
//	NRF24L01_TxMode();
	
		CONFIG = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
		if ( CONFIG != 0X0E)
		{
			NRF24L01_Init_();
	
		}
	
	
		Status = NRF24L01_ReadStatus();
		if (Status&0X20)
		{
			NRF24L01_ClearTxFIFO();
			NRF24L01_WriteRge(NRF24L01_STATUS,0X20);
			
			return 1;
		}
		else if (Status&0X10)
		{
			NRF24L01_ClearTxFIFO();
			NRF24L01_WriteRge(NRF24L01_STATUS,0X10);
//			NRF24L01_Init_();
			
			return 2;
		}

	
	
}



uint8_t NRF24L01_Receive(void)
{
//	NRF24L01_RxMode();
	uint8_t CONFIG;
	uint8_t Status;
	Status = NRF24L01_ReadStatus();
	CONFIG = NRF24L01_ReadReg(NRF24L01_CONFIG);

		if ( CONFIG != 0X0F)
		{
			NRF24L01_Init_();
	
		}
	
	
	CONFIG = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
	if (Status&0X40)
	{
		NRF24L01_ReadRxFIFO(NRF24L01_ReceiveByte,NRF24L01_RX_SendByte);
		NRF24L01_WriteRge(NRF24L01_STATUS,0X40);
		NRF24L01_ClearRxFIFO();
		
		timeout = 1000;
		
		return 0;
		
	}
	if (timeout-- == 0) 
		{
			timeout = 1000;
			NRF24L01_Init_();
			NRF24L01_RxMode();
			return 1;  
		}

//	My_spi_ReadWrite(NRF24L01_STATUS,0X7E);
		
		
		
	return 1;
}











