#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* 必须包含这个头文件，否则 SystemCoreClock 报错 */
#include "stm32f4xx.h"

/******************************************************************************/
/* 基础硬件配置                                                               */
/******************************************************************************/

/* 使用系统核心时钟变量 (100MHz) */
#define configCPU_CLOCK_HZ                       ( ( unsigned long ) SystemCoreClock )

/* 1000Hz 心跳 (1ms 一个 tick)，飞控标准配置 */
#define configTICK_RATE_HZ                       ( ( TickType_t ) 1000 )

/* 开启抢占式调度 */
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1 /* STM32有特殊指令，开启优化 */
#define configUSE_TICKLESS_IDLE                  0 /* 飞控不需要低功耗模式 */

#define configMAX_PRIORITIES                     ( 5 )
#define configMINIMAL_STACK_SIZE                 ( ( unsigned short ) 128 )

/* 🔴 关键修改：堆内存大小 75KB (F411 有 128KB RAM，给 RTOS 75KB 很安全) */
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 75 * 1024 ) )

#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1

/* 🟢 关键修改：开启 FPU 支持 */
#define configENABLE_FPU                         1
#define configENABLE_MPU                         0

/******************************************************************************/
/* 中断优先级配置 (Cortex-M 核心最重要部分)                                   */
/******************************************************************************/

/* 最低优先级 (15) */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          0xF

/* RTOS 可管理的最高优先级 (5) - 你的串口/定时器中断优先级必须 >= 5 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5

/* 内核中断优先级设置 (不要改) */
#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << 4 )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << 4 )

/******************************************************************************/
/* 钩子函数 (调试用)                                                          */
/******************************************************************************/
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             0
#define configCHECK_FOR_STACK_OVERFLOW           0 /* 调试时可设为2，正式飞设为0 */

/******************************************************************************/
/* API 函数开关                                                               */
/******************************************************************************/
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1

/* 断言配置 (开发阶段非常有用，死机时能卡住) */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/******************************************************************************/
/* 中断函数映射 (把 FreeRTOS 函数接到 STM32 中断向量表)                       */
/******************************************************************************/
#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
