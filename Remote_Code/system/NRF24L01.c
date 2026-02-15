#include "stm32f10x.h"                  // Device header
#include "NRF24L01_Define.h" 
#include "Delay.h"



uint8_t RxAddress[5]={0X22,0X22,0X22,0X22,0X22};
uint8_t TxAddress[5]={0X22,0X22,0X22,0X22,0X22};

uint8_t NRF24L01_SendByte[32];
uint8_t NRF24L01_ReceiveByte[32];

#define NRF24L01_RX_SendByte   32

/*引脚定义    PA3---PA7*/


uint32_t timeout = 1000;


void NRF24L01_W_CE(uint8_t Val)
{
	
	GPIO_WriteBit(GPIOA,GPIO_Pin_3,(BitAction)Val);

}


void NRF24L01_W_CSN(uint8_t Val)
{
	
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,(BitAction)Val);

}

void NRF24L01_W_SCK(uint8_t Val)
{
	
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,(BitAction)Val);

}

void NRF24L01_W_MOSI(uint8_t Val)
{
	
	GPIO_WriteBit(GPIOA,GPIO_Pin_7,(BitAction)Val);

}

uint8_t NRF24L01_R_MISO(void)
{
	
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);

}


uint8_t NRF24L01_R_IRQ(void)
{
	
	return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);

}








void NRF24L01_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructuer;
	GPIO_Initstructuer.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Initstructuer.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;
	GPIO_Initstructuer.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructuer);
	
	
	GPIO_Initstructuer.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Initstructuer.GPIO_Pin = GPIO_Pin_6;
	GPIO_Initstructuer.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructuer);
	
	GPIO_Initstructuer.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Initstructuer.GPIO_Pin = GPIO_Pin_11;
	GPIO_Initstructuer.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructuer);
	
	
	NRF24L01_W_CE(0);
	NRF24L01_W_CSN(1);
	NRF24L01_W_SCK(0);	
	NRF24L01_W_MOSI(0);


}




		//软件SPI



uint8_t NRF24L01_SPI_Swap(uint8_t Byte)
{
	
	uint8_t i;
	for (i=0 ; i<8 ; i++)
	{
		//模式0
		//移入数据
		if (Byte&0x80)
		{
			NRF24L01_W_MOSI(1);
		}
		else
		{
			NRF24L01_W_MOSI(0);
		}
		Byte <<=1;
		//拉高时钟线
		NRF24L01_W_SCK(1);
		//读取MISO
		if (NRF24L01_R_MISO())
		{
			Byte |= 0x01; 
		}
		else    
		{
			Byte &= ~0x01;
		}
		//拉低时钟线		
		NRF24L01_W_SCK(0);
	}
	return Byte;
}


uint8_t NRF24L01_SPI_Swap_S(uint8_t Byte,uint8_t L)
{
	
	uint8_t i;
	for (i=0 ; i<L ; i++)
	{
		//模式0
		//移入数据
		if (Byte&0x80)
		{
			NRF24L01_W_MOSI(1);
		}
		else
		{
			NRF24L01_W_MOSI(0);
		}
		Byte <<=1;
		//拉高时钟线
		NRF24L01_W_SCK(1);
		//读取MISO
		if (NRF24L01_R_MISO())
		{
			Byte |= 0x01; 
		}
		else    
		{
			Byte &= ~0x01;
		}
		//拉低时钟线		
		NRF24L01_W_SCK(0);
	}
	return Byte;
}







void NRF24L01_WriteRge(uint8_t Reg,uint8_t Data)
{
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_W_REGISTER | Reg);
	 NRF24L01_SPI_Swap(Data);
	 NRF24L01_W_CSN(1);

}


uint8_t NRF24L01_ReadReg(uint8_t Reg)
{
	 uint8_t Data;
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_R_REGISTER | Reg);
	 Data = NRF24L01_SPI_Swap(NRF24L01_NOP);
	 NRF24L01_W_CSN(1);	 
	 return Data;

}

void NRF24L01_WriteRgeS(uint8_t Reg,uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_W_REGISTER | Reg);
	for (i=0;i<count;i++)
	{
		 NRF24L01_SPI_Swap(DataArray[i]);
	}
	 NRF24L01_W_CSN(1);

}


void NRF24L01_ReadRegS(uint8_t Reg,uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_R_REGISTER | Reg);
	 for (i=0;i<count;i++)
     {
		DataArray[i] = NRF24L01_SPI_Swap(NRF24L01_NOP);
	 }
	 NRF24L01_W_CSN(1);	 

}


void NRF24L01_WriteTxFIFO(uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_W_TX_PAYLOAD);
	for (i=0;i<count;i++)
	{
		 NRF24L01_SPI_Swap(DataArray[i]);		
		
	}
	 NRF24L01_W_CSN(1);

}


void NRF24L01_ReadRxFIFO(uint8_t* DataArray,uint8_t count)
{
	 uint8_t i;
	 uint16_t temp;
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_R_RX_PAYLOAD);
	 for (i=0;i<count;i++)
     {
		DataArray[i] = NRF24L01_SPI_Swap(NRF24L01_NOP);
		 
		 
	 }
	 NRF24L01_W_CSN(1);	 

}

void NRF24L01_ClearRxFIFO(void)
{
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_FLUSH_RX);
	 NRF24L01_W_CSN(1);	 

}


void NRF24L01_ClearTxFIFO(void)
{
	 NRF24L01_W_CSN(0);
	 NRF24L01_SPI_Swap(NRF24L01_FLUSH_TX);
	 NRF24L01_W_CSN(1);	 

}


uint8_t NRF24L01_ReadStatus(void)
{
	 uint8_t Status;
	 NRF24L01_W_CSN(0);
	 Status = NRF24L01_SPI_Swap(NRF24L01_NOP);
	 NRF24L01_W_CSN(1);	 
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
	
	
//	Delay_us (200);
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
	
//	Delay_us(1500);
	
	 NRF24L01_W_CE(1);
}


void NRF24L01_Init()
{
	
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
	
	Delay_ms(2);
	
	NRF24L01_RxMode();
	
	
	
	
	
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
	
//		CONFIG = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
	uint32_t timeout = 0;
	
	while (timeout < 50000)
	{
		
		timeout++;
		
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
			NRF24L01_Init();
			
			return 2;
		}

	}
	
	NRF24L01_ClearTxFIFO();
    NRF24L01_WriteRge(NRF24L01_STATUS, 0x70); // 清除所有标志
    return 3;
	
}



uint8_t NRF24L01_Receive(void)
{
	
	uint8_t CONFIG;
	uint8_t Status;
	Status = NRF24L01_ReadStatus();
	
	
	
	if (Status&0X40)
	{
		NRF24L01_ReadRxFIFO(NRF24L01_ReceiveByte,NRF24L01_RX_SendByte);
		NRF24L01_WriteRge(NRF24L01_STATUS,0X40);
		NRF24L01_ClearRxFIFO();
		
		return 0;
		
	}
		
		return 1;
}



uint8_t NRF24L01_Receive_Link(void)
{
	
	uint8_t CONFIG;
	uint8_t Status;
	Status = NRF24L01_ReadStatus();
	
	
	
	if (Status&0X40)
	{
		NRF24L01_ReadRxFIFO(NRF24L01_ReceiveByte,NRF24L01_RX_SendByte);
		NRF24L01_WriteRge(NRF24L01_STATUS,0X40);
		NRF24L01_ClearRxFIFO();
		
		return 0;
		
	}


		
		return 1;
		
}







