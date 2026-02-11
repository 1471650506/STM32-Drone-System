#include "stm32f10x.h"                  // Device header

uint16_t ADC_V[5]={0,0,0,0,0};


void DMA_user_Init(void)
{
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	
	
	
	
	DMA_InitTypeDef DMA_Initstructure;
	DMA_Initstructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//外设地址
	DMA_Initstructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//字符大小
	DMA_Initstructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//地址自增
	DMA_Initstructure.DMA_MemoryBaseAddr = (uint32_t)ADC_V;
	DMA_Initstructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_Initstructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Initstructure.DMA_BufferSize = 5;//缓冲器大小
	DMA_Initstructure.DMA_DIR = DMA_DIR_PeripheralSRC;//传输方向,外设到内存
	DMA_Initstructure.DMA_M2M = DMA_M2M_Disable;//触发方式,软件还是硬件
	DMA_Initstructure.DMA_Mode = DMA_Mode_Circular;//自动重装,循环
	DMA_Initstructure.DMA_Priority = DMA_Priority_Medium;//优先级
	DMA_Init (DMA1_Channel1,&DMA_Initstructure);
	
	
	
	DMA_Cmd(DMA1_Channel1,ENABLE);
	
	
	
	
	
	
	
}


















