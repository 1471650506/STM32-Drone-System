#ifndef __ADC_H
#define __ADC_H

void ADC1_CH0_Init(void);

uint8_t Battery_GetPercent(float vbat);
float Battery_GetVoltage(void);



#endif
