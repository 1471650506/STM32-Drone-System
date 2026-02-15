/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include <stdio.h>

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);

volatile uint32_t HardFault_Regs[8];

typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} HardFaultRegs_t;

volatile HardFaultRegs_t faultRegs; 


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */







void HardFault_Handler_C(uint32_t *hardfault_args)
{
    /* 提取堆栈里的寄存器值到全局数组 */
    HardFault_Regs[0] = hardfault_args[0]; // R0
    HardFault_Regs[1] = hardfault_args[1]; // R1
    HardFault_Regs[2] = hardfault_args[2]; // R2
    HardFault_Regs[3] = hardfault_args[3]; // R3
    HardFault_Regs[4] = hardfault_args[4]; // R12
    HardFault_Regs[5] = hardfault_args[5]; // LR
    HardFault_Regs[6] = hardfault_args[6]; // PC (这是最重要的！死机位置)
    HardFault_Regs[7] = hardfault_args[7]; // PSR

    /* 死循环，等待你查看 HardFault_Regs 数组 */
    while (1)
    {
    }
}


__asm void HardFault_Handler(void)
{
    IMPORT  HardFault_Handler_C
    
    /* 判断使用的是 MSP 还是 PSP 堆栈 */
    TST     lr, #4
    ITE     EQ
    MRSEQ   r0, MSP
    MRSNE   r0, PSP
    
    /* 跳转到 C 函数，并将堆栈指针作为参数 R0 传入 */
    B       HardFault_Handler_C
}



/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//	vPortSVCHandler();
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//	// 调用 FreeRTOS 的任务切换函数
//    xPortPendSVHandler();
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
////  TimingDelay_Decrement();
//	
//	// 调用 FreeRTOS 的心跳函数 (这就是那个闹钟！)
//    xPortSysTickHandler();
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


