#include <stm32f4xx.h>
#include "PID.h"
#include "NRF24L01.h"
#include "imu.h"
#include "MPU6050.h"
#include "Main.h"
#include "Motor.h"
#include "MY_USART1.h"




extern FloatPacket PID_ROL_Rate_p,PID_ROL_Rate_i,PID_ROL_Rate_d;
extern FloatPacket PID_PIT_Rate_p,PID_PIT_Rate_i,PID_PIT_Rate_d;
extern FloatPacket PID_YAW_Rate_p,PID_YAW_Rate_i,PID_YAW_Rate_d;

//外环pid
extern FloatPacket PID_ROL_Angle_p,PID_ROL_Angle_i,PID_ROL_Angle_d;
extern FloatPacket PID_PIT_Angle_p,PID_PIT_Angle_i,PID_PIT_Angle_d;
extern FloatPacket PID_YAW_Angle_p,PID_YAW_Angle_i,PID_YAW_Angle_d;

//角度修正
extern FloatPacket roll,pit,yaw;

float ROLL_OFFSET   = 0.0f	; //修正值
float PITCH_OFFSET  = 0.0f ; //修正值


//角度环PID 
PID_TYPE PID_ROL_Angle;
PID_TYPE PID_PIT_Angle;
PID_TYPE PID_YAW_Angle;
//角速度环PID 
PID_TYPE PID_ROL_Rate;
PID_TYPE PID_PIT_Rate;
PID_TYPE PID_YAW_Rate;
//高度环PID 
PID_TYPE PID_ALT_Rate;
PID_TYPE PID_ALT;

int16_t 	Moto_PWM_1 = 100,
			Moto_PWM_2 = 100,
			Moto_PWM_3 = 100,
			Moto_PWM_4 = 100;

/*
float target   目标值
float measure  测量值
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
*/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure) 
{
	PID->Error    =   target - measure;
	PID->Differ   =   PID->Error - PID->PreError;
	
	if ((Receive.off ==0X0A) && (Receive.POWER>=200))
	{
		if (measure > (PID->Ilimit) || measure < -PID->Ilimit)
		{
			PID->Ilimit_flag = 0;	
		}
		else 
		{
			PID->Ilimit_flag = 1;
			
			PID->Integral +=  PID->Error;
			
			if (PID->Integral  >  PID->Irang)
			{
				PID->Integral = PID->Irang;
			}
			if (PID->Integral  <  -PID->Irang)
			{
				PID->Integral = -PID->Irang;
			}
			
			
		}
		
		
		
	}
	else
	{
		PID->Integral = 0;	
	}	
	


	PID->Pout  = PID->P * PID->Error;
	PID->Iout  = PID->Ilimit_flag*PID->I * PID->Integral;
	PID->Dout  = PID->D * PID->Differ;
	
	PID->OutPut = PID->Pout + PID->Iout + PID->Dout;
	

	

	PID->PreError = PID->Error;
	
}



void PID_structuer_Init(void)
{
	//ROLL轴
	PID_ROL_Rate.Ilimit_flag = 0; //Roll轴角速度积分的分离标志
	PID_ROL_Rate.Ilimit = 150;    //Roll轴角速度积分范围
	PID_ROL_Rate.Irang = 7000;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_ROL_Angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
	PID_ROL_Angle.Ilimit = 35;    //Roll轴角度积分范围
	PID_ROL_Angle.Irang = 200;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	
	

	
	//PITCH轴
	PID_PIT_Rate.Ilimit_flag = 0; //Pitch轴角速度积分的分离标志
	PID_PIT_Rate.Ilimit = 150;    //Pitch轴角速度积分范围
	PID_PIT_Rate.Irang = 7000;    //Pitch轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_PIT_Angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
	PID_PIT_Angle.Ilimit = 35;    //Roll轴角度积分范围
	PID_PIT_Angle.Irang = 200;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
		
	
	//YAW轴
	PID_YAW_Rate.Ilimit_flag = 0; //Yaw轴角速度积分的分离标志
	PID_YAW_Rate.Ilimit = 150;    //Yaw轴角速度积分范围
	PID_YAW_Rate.Irang = 1200;    //Yaw轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_YAW_Angle.Ilimit_flag = 0;//Yaw轴角度积分的分离标志
	PID_YAW_Angle.Ilimit = 35;    //Yaw轴角度积分范围
	PID_YAW_Angle.Irang = 200;    //Yaw轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  	
	
	
	//高度环
	PID_ALT_Rate.Ilimit_flag = 0;
	PID_ALT_Rate.Ilimit = 0;
	PID_ALT_Rate.Irang = 0;
	PID_ALT.Ilimit_flag = 0;
	PID_ALT.Ilimit = 100;
	PID_ALT.Irang = 200;	
	
	
	
	//角速度环PID 
	
	PID_ROL_Rate.P = PID_ROL_Rate_p.f;
	PID_PIT_Rate.P = PID_PIT_Rate_p.f;
	PID_YAW_Rate.P = PID_YAW_Rate_p.f;
	
	PID_ROL_Rate.I = PID_ROL_Rate_i.f;
	PID_PIT_Rate.I = PID_PIT_Rate_i.f;
	PID_YAW_Rate.I = PID_YAW_Rate_i.f;
	
	PID_ROL_Rate.D = PID_ROL_Rate_d.f;
	PID_PIT_Rate.D = PID_PIT_Rate_d.f;
	PID_YAW_Rate.D = PID_YAW_Rate_d.f;
   
	//角度环PID 
	PID_ROL_Angle.P = PID_ROL_Angle_p.f;
	PID_PIT_Angle.P = PID_PIT_Angle_p.f;
	PID_YAW_Angle.P = PID_YAW_Angle_p.f;
	             
	PID_ROL_Angle.I = PID_ROL_Angle_i.f;
	PID_PIT_Angle.I = PID_PIT_Angle_i.f;
	PID_YAW_Angle.I = PID_YAW_Angle_i.f;
	             
	PID_ROL_Angle.D = PID_ROL_Angle_d.f;
	PID_PIT_Angle.D = PID_PIT_Angle_d.f;
    PID_YAW_Angle.D = PID_YAW_Angle_d.f;
   
   
   
    ROLL_OFFSET = roll.f;
	PITCH_OFFSET = pit.f;
}


void control_pid(EulerAngles_t* att_in,MPU6050_type* G,NRF_Receive* Receive)
{
	
	EulerAngles_t Measure_Angle,Target_Angle;
	Measure_Angle.roll = att_in->roll; 
	Measure_Angle.pitch = att_in->pitch; 
	Measure_Angle.yaw = att_in->yaw; 
	Target_Angle.roll = (float)((Receive->roll-1500)/12.0f);
	Target_Angle.pitch = (float)((Receive->pitch-1500)/12.0f);
	Target_Angle.yaw = (float)((1500-Receive->yaw)/12.0f);
	
	Target_Angle.roll  += ROLL_OFFSET;
    Target_Angle.pitch += PITCH_OFFSET;
	
	
	usart_s [4] = G->X ;
	usart_s [5] = G->Y ;	
	
		//角度环
	PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.roll,Measure_Angle.roll);//ROLL角度环PID （输入角度 输出角速度）
	PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pitch,Measure_Angle.pitch);//PITH角度环PID （输入角度 输出角速度）
//	PID_Postion_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);//YAW角度环PID  （输入角度 输出角速度）
	
	//角速度环
	
	PID_Postion_Cal(&PID_ROL_Rate, PID_ROL_Angle.OutPut, G->X); //ROLL角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_PIT_Rate, PID_PIT_Angle.OutPut, G->Y); //PITH角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_YAW_Rate,Target_Angle.yaw*PID_YAW_Angle.P,G->Z); //YAW角速度环PID （输入角度，输出电机控制量）
	
	
	
	
//	usart_s [0] = PID_ROL_Angle.OutPut;
//	usart_s [1] = (G->Y);
//	usart_s [2] = PID_ROL_Rate.OutPut;
	usart_s [6] = PID_PIT_Rate.Pout;
	usart_s [7] = PID_PIT_Rate.Iout;
	usart_s [8] = PID_PIT_Rate.OutPut;
	
	
	if(Receive->POWER>180)//当油门大于180时动力分配才生效
	{                                                                                 
		
		Moto_PWM_1 = Receive->POWER - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;		
		Moto_PWM_2 = Receive->POWER + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;		
		Moto_PWM_3 = Receive->POWER - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
		Moto_PWM_4 = Receive->POWER + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;		
	
		
		
	} 
	else 
	{
		Moto_PWM_1 = 0;
		Moto_PWM_2 = 0;
		Moto_PWM_3 = 0;
		Moto_PWM_4 = 0;
	}
	
	Moto_PWM_1 = ccr_xf(Moto_PWM_1);
	Moto_PWM_2 = ccr_xf(Moto_PWM_2);
	Moto_PWM_3 = ccr_xf(Moto_PWM_3);
	Moto_PWM_4 = ccr_xf(Moto_PWM_4);
	
	usart_s [0] = Moto_PWM_1 ;
	usart_s [1] = Moto_PWM_2 ;
	usart_s [2] = Moto_PWM_3 ;
	usart_s [3] = Moto_PWM_4 ;
	
   TIM_CCR(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4); 

	
	
}












