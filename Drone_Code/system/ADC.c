#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>

void ADC1_CH0_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    /* 1. 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    /* 2. PA0 -> ADC IN0 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 3. ADC 通用配置 */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    
    /* 4. ADC1 配置 */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* 5. 通道选择：IN0, 采样时间至少 144cycles（分压 100K 需要长时间） */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
    
    /* 6. 启动 ADC */
    ADC_Cmd(ADC1, ENABLE);

    /* 7. 软件开始第一次转换 */
    ADC_SoftwareStartConv(ADC1);
}






uint16_t ADC1_Read(void)
{
    ADC_SoftwareStartConv(ADC1);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    return ADC_GetConversionValue(ADC1);
}






float Battery_GetVoltage(void)
{
    uint16_t raw = ADC1_Read();
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



