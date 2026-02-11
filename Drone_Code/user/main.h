/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Header for main.c module
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
  
#include <stm32f4xx.h>

  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


extern uint8_t SCL,SDA;



typedef struct 
{
	uint8_t  START ;//开始帧
	uint8_t  CONTROL;//读写控制位
	
	float roll;   // 滚转角
	float POWER;    // 油门
	float pitch;  // 俯仰角 
	float yaw;    // 偏航角
	
	uint8_t  off ;//开关
	
	uint8_t  STOP ;//结束帧
	
	
}NRF_Receive;
	

extern  NRF_Receive Receive;


 typedef union {
    float f;          // 浮点视角 (存数据用)
    unsigned char b[4]; // 字节视角 (发数据用)
} FloatPacket;

//内环pid
extern FloatPacket PID_ROL_Rate_p,PID_ROL_Rate_i,PID_ROL_Rate_d;
extern FloatPacket PID_PIT_Rate_p,PID_PIT_Rate_i,PID_PIT_Rate_d;
extern FloatPacket PID_YAW_Rate_p,PID_YAW_Rate_i,PID_YAW_Rate_d;

//外环pid
extern FloatPacket PID_ROL_Angle_p,PID_ROL_Angle_i,PID_ROL_Angle_d;
extern FloatPacket PID_PIT_Angle_p,PID_PIT_Angle_i,PID_PIT_Angle_d;
extern FloatPacket PID_YAW_Angle_p,PID_YAW_Angle_i,PID_YAW_Angle_d;

//角度修正
extern FloatPacket roll,pit,yaw;
 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);

#endif /* __MAIN_H */

