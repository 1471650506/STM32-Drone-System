#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "Delay.h"


volatile uint8_t pid_execute_flag = 0;
/**
  * @brief  初始化TIM3的PWM输出，频率为1kHz，默认占空比50%。
  * @param  无
  * @retval 无
  */
void TIM3_PWM_Init(void)
{

    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    // 2. 初始化GPIO (复用功能，上拉，高速)
	
	GPIO_InitTypeDef GPIO_InitStruct;

	
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // 复用功能
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // 高速

    // 2.1 配置TIM3通道1和2对应的GPIOA引脚
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); // PA6 -> TIM3_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); // PA7 -> TIM3_CH2

    // 2.2 配置TIM3通道3和4对应的GPIOB引脚
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); // PB0 -> TIM3_CH3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); // PB1 -> TIM3_CH4

    // 3. 配置TIM3时基单元
    // 假设系统时钟为84MHz，目标PWM频率为1kHz
    // 定时器时钟 = 84MHz / (PSC+1) = 84MHz / 84 = 1MHz
    // 溢出频率 = 1MHz / (ARR+1) = 1MHz / 1000 = 1kHz
	
	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
    TIM_TimeBaseInitStruct.TIM_Prescaler = 83;               // 预分频值PSC
    TIM_TimeBaseInitStruct.TIM_Period = 999;                 // 自动重装载值ARR
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // 4. 配置TIM3为PWM模式
	
    TIM_OCInitTypeDef TIM_OCInitStruct;
	
	
	
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;          // PWM模式1[citation:2][citation:7]
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;  // 高电平为有效极性
    TIM_OCInitStruct.TIM_Pulse = 0;                       // 初始占空比(CCR值)

    // 4.1 初始化四个PWM通道
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);  // 通道1
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);  // 通道2
    TIM_OC3Init(TIM3, &TIM_OCInitStruct);  // 通道3
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);  // 通道4

    // 5. 使能TIM3各通道的预装载寄存器(CCR)
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 6. 使能TIM3的ARR预装载寄存器
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // 7. 使能TIM3计数器
    TIM_Cmd(TIM3, ENABLE);
}










void TIM3_SetPWM_Duty(uint8_t Channel, uint16_t DutyCycle)
{
    // 确保CCR值不超过ARR，防止异常
    if(DutyCycle > TIM3->ARR) {
        DutyCycle = TIM3->ARR;
    }
    
    switch(Channel) {
        case 1:
            TIM_SetCompare1(TIM3, DutyCycle); // 更新通道1的CCR1[citation:7]
            break;
        case 2:
            TIM_SetCompare2(TIM3, DutyCycle); // 更新通道2的CCR2
            break;
        case 3:
            TIM_SetCompare3(TIM3, DutyCycle); // 更新通道3的CCR3
            break;
        case 4:
            TIM_SetCompare4(TIM3, DutyCycle); // 更新通道4的CCR4[citation:2]
            break;
        default:
            // 可在此添加错误处理
            break;
    }
}

int16_t ccr_xf(int16_t ccr)
{
	if (ccr <= 0)
	{
		
		ccr = 0;
		
	}
	
	if (ccr >= 999)
	{
		
		ccr = 999;
		
	}
	
	return ccr;
	
}
	





void TIM_CCR(uint16_t Moto_PWM_1,uint16_t Moto_PWM_2,uint16_t Moto_PWM_3,uint16_t Moto_PWM_4)
{
	
	TIM_SetCompare1(TIM3, Moto_PWM_1);
	TIM_SetCompare2(TIM3, Moto_PWM_2);
	TIM_SetCompare3(TIM3, Moto_PWM_3);
	TIM_SetCompare4(TIM3, Moto_PWM_4);
	
}


void TIM_CCR_init(void)
{
	
	TIM_SetCompare1(TIM3, 100);
	
	Delay_s(1);
	
	TIM_SetCompare2(TIM3, 100);
	
	Delay_s(1);
	
	TIM_SetCompare3(TIM3, 100);
	
	Delay_s(1);
	
	TIM_SetCompare4(TIM3, 100);
	
	Delay_s(1);

}






void TIM4_Int_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 2. 配置时基单元
    // 目标: 1MHz 的计数频率 (1us 跳一次)
    // APB1 时钟预分频后如果是 84MHz:
    // Prescaler = 84 - 1 = 83
    TIM_TimeBaseStructure.TIM_Prescaler = 83; 
    
    // 目标: 2.5ms 触发一次中断 (400Hz)
    // 2500us / 1us = 2500
    // Period (ARR) = 2500 - 1 = 2499
    TIM_TimeBaseStructure.TIM_Period = 1999; 
    
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 3. 开启 TIM4 更新中断 (Update Interrupt)
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // 4. 配置 NVIC 中断优先级
    // PID 控制属于核心任务，优先级应该较高 (PreemptionPriority 小)
    // 但要比通信中断或紧急保护低一点，这里设为 1
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 使能 TIM4
    TIM_Cmd(TIM4, ENABLE);
}




void TIM4_IRQHandler(void)
{
    // 检查是否是 TIM4 更新中断
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        
        // 置位标志，通知主循环执行 PID
        pid_execute_flag = 1; 
    }
}











