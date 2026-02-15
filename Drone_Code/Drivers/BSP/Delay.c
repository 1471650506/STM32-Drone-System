//#include "stm32f10x.h"
#include <stm32f4xx.h>




#define  DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define  DWT_CR      *(volatile uint32_t *)0xE0001000
#define  DEMCR       *(volatile uint32_t *)0xE000EDFC

void DWT_Init(void)
{
    // 1. 开启 TRCENA 位 (Trace Enable)，位于 DEMCR 的第 24 位
    DEMCR |= (1 << 24); 
    
    // 2. 清空计数器
    DWT_CYCCNT = 0;
    
    // 3. 开启 CYCCNTENA 位，位于 DWT_CTRL 的第 0 位
    DWT_CR |= (1 << 0);
}



void Delay_us(uint32_t xus)
{
	uint32_t ticks = xus * (SystemCoreClock / 1000000);
    uint32_t t_old = DWT_CYCCNT;
    while ((DWT_CYCCNT - t_old) < ticks);
}

void Delay_ms(uint32_t ms)
{
    for (int i = 0; i < ms; i++)
    {
        Delay_us(1000);
    }
}
