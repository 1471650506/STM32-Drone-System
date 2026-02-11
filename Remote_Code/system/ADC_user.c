#include "stm32f10x.h"                  // Device header
#include "DMA_user.h"
#include "Oled_menu.h"

uint16_t  ADC_V_Calibration[4];
uint16_t  ADC_V_Send[4];

void ADC_user_Init(void)
{
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//分频


	GPIO_InitTypeDef GPIO_initstructure;//创建结构体
	GPIO_initstructure.GPIO_Mode = GPIO_Mode_AIN;//设置为模拟量输入
	GPIO_initstructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2;//PA0为电池模拟量,PA1为摇杆模拟量
	GPIO_initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initstructure);
	
	
	GPIO_initstructure.GPIO_Mode = GPIO_Mode_AIN;//设置为模拟量输入
	GPIO_initstructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1;//PB0为摇杆模拟量
	GPIO_initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_initstructure);
	
	
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);//锂电池
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_55Cycles5);//摇杆PA1
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,3,ADC_SampleTime_55Cycles5);//摇杆PA2
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,4,ADC_SampleTime_55Cycles5);//摇杆PB0
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,5,ADC_SampleTime_55Cycles5);//摇杆PB1

	
	
	
	
	
	
	
	ADC_InitTypeDef ADC_Initstructure;
	ADC_Initstructure.ADC_Mode = ADC_Mode_Independent;  //ADC模式为单adc
	ADC_Initstructure.ADC_DataAlign = ADC_DataAlign_Right;//数据对齐
	ADC_Initstructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//软件触发
	ADC_Initstructure.ADC_ContinuousConvMode = ENABLE;//连续模式
	ADC_Initstructure.ADC_ScanConvMode = ENABLE;//扫描模式
	ADC_Initstructure.ADC_NbrOfChannel = 5;//通道数
	ADC_Init(ADC1,&ADC_Initstructure);
	
	
	
	
	
	
	DMA_user_Init();
	
	
	ADC_DMACmd(ADC1,ENABLE);
	
	
	ADC_Cmd(ADC1,ENABLE);
	
	
	
	/*  ADC校准  */
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	
	
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	
	
	
	
	
	
	

}





/*
去30个adc的值求平均值来校准adc


*/
void  ADC_V_ResetCalibration(void)	
{
	uint8_t count = 0;
	uint32_t D0 = 0,D1 = 0,D2 = 0,D3 = 0;
	
	while (1)
	{
	
	if (count == 0)
	{
		D0 = 0;
		D1 = 0;
		D2 = 0;
		D3 = 0;
		count = 0;
	}
	
	D0 += (1000 + ADC_V[1]*1000/4096 );
	D1 += (1000 + ADC_V[2]*1000/4096 );
	D2 += (1000 + ADC_V[3]*1000/4096 );
	D3 += (1000 + ADC_V[4]*1000/4096 );
	
	count ++;
	
	if (count >= 10)
	{
		ADC_V_Calibration[0] = D0/count;
		ADC_V_Calibration[1] = D1/count;
		ADC_V_Calibration[2] = D2/count;
		ADC_V_Calibration[3] = D3/count;
		
		count = 0;
		
		break;
	}
	
	}
}


void ADC_Send(void)
{
	
	ADC_V_Send[0] = ADC_V[1]*1000/4096;//油门
//	ADC_V_Send[0] = (ADC_V_Send[0]<=1000)?1000:ADC_V_Send[0];
//	ADC_V_Send[0] = (ADC_V_Send[0]>=1700)?1700:ADC_V_Send[0];
	
	ADC_V_Send[1] = 1500+((1000+ADC_V[2]*1000/4096)-ADC_V_Calibration[1]);
	ADC_V_Send[1] = (ADC_V_Send[1]<=1000)?1000:ADC_V_Send[1];
	ADC_V_Send[1] = (ADC_V_Send[1]>=2000)?2000:ADC_V_Send[1];
	
	ADC_V_Send[2] = 1500+((1000+ADC_V[3]*1000/4096)-ADC_V_Calibration[2]);
	ADC_V_Send[2] = (ADC_V_Send[2]<=1000)?1000:ADC_V_Send[2];
	ADC_V_Send[2] = (ADC_V_Send[2]>=2000)?2000:ADC_V_Send[2];
	
	ADC_V_Send[3] = 1500+((1000+ADC_V[4]*1000/4096)-ADC_V_Calibration[3]);
	ADC_V_Send[3] = (ADC_V_Send[3]<=1000)?1000:ADC_V_Send[3];
	ADC_V_Send[3] = (ADC_V_Send[3]>=2000)?2000:ADC_V_Send[3];
	
	
	
	oled_menu_structuer[3] =ADC_V_Send[0];
	
	
	
	
}

	

float Battery_GetVoltage(void)
{
    uint16_t raw = ADC_V[0];
    float v_adc = raw * 3.3f / 4095.0f;
    float v_bat = v_adc * 2; // 分压系数2

    return v_bat;
}

uint8_t Battery_GetPercent(float vbat)
{

	
    if (vbat >= 4.20f) return 100;
    if (vbat <= 3.30f) return 0;

    return (uint8_t)((vbat - 3.30f) * 100.0f / (4.20f - 3.30f));
}





