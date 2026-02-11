#ifndef __MOTOR_H
#define __MOTOR_H




extern volatile uint8_t pid_execute_flag;

void TIM3_PWM_Init(void);


void TIM3_SetPWM_Duty(uint8_t Channel, uint16_t DutyCycle);

void TIM_CCR(uint16_t Moto_PWM_1,uint16_t Moto_PWM_2,uint16_t Moto_PWM_3,uint16_t Moto_PWM_4);

void TIM_CCR_init(void);

int16_t ccr_xf(int16_t ccr);

void TIM4_Int_Init(void);


void TIM4_IRQHandler(void);


#endif
