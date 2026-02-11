#ifndef __ADC_user_H
#define __ADC_user_H


extern uint16_t  ADC_V_Calibration[4];
extern uint16_t  ADC_V_Send[4];

void ADC_user_Init(void);

void  ADC_V_ResetCalibration(void);	


void ADC_Send(void);
float Battery_GetVoltage(void);

uint8_t Battery_GetPercent(float vbat);



#endif
