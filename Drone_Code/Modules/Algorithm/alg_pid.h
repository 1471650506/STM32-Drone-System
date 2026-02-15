#ifndef __PID_H
#define __PID_H
#include <stm32f4xx.h>
#include "dev_mpu6050.h"
#include "alg_imu.h"
#include "bsp_usart.h"
#include "dev_nrf24l01.h"
#include "stdio.h"
#include "alg_pid.h"
#include "bsp_adc.h"
#include "dev_motor.h"
#include "Main.h"

typedef struct {
    float P;                  //比例项系数
    float I;                  //积分项系数
    float D;                  //微分项系数
	float Error;              //比例项Ek,输入与反馈的误差   
	float Integral;           //积分项Sk,比例项Ek的和
	float Differ;             //微分项Dk,本次误差减去上一次误差 
	float PreError;           //上一次误差
	float Ilimit;             //积分分离     
	float Irang;              //积分限幅    
	int   Ilimit_flag;        //积分分离 标志   
	float Pout;               //比例项输出
    float Iout;               //积分项输出
    float Dout;               //微分项输出
	float OutPut;             //PID控制器输出     
} PID_TYPE;




void PID_Postion_Cal(PID_TYPE*PID,float target,float measure) ;


void PID_structuer_Init(void);

void control_pid(EulerAngles_t* att_in,MPU6050_type* G,NRF_Receive* Receive);





#endif











