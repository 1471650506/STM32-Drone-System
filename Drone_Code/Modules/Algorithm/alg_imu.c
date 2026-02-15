#include <stm32f4xx.h>
#include "dev_mpu6050.h"
#include "math.h"
#include "Main.h"
#include "bsp_usart.h"



static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


float LPF(float old, float new, float alpha)
{
    return alpha * new + (1 - alpha) * old;
}











#define kp 1.50f 
#define ki 0.005f 
#define halft 0.001f 

typedef struct {
    float roll;   // 滚转角
    float pitch;  // 俯仰角  
    float yaw;    // 偏航角
} EulerAngles_t;


float q0 =1,q1 = 0,q2 = 0,q3 = 0;
float exint = 0,eyint = 0,ezint = 0;
	
static float gx_f = 0, gy_f = 0, gz_f = 0;

int16_t AX, AY, AZ, GX, GY, GZ;	


void IMU(EulerAngles_t* euler,MPU6050_type* G)
{
//	uint8_t i;
//	MPU6050_Init();
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	
	float gyro_raw_x =0;
	float gyro_raw_y =0;
	float gyro_raw_z =0;
	
	static float ax_f, ay_f, az_f;

	ax_f = LPF(ax_f, AX, 0.4f); // α 越小滤波越强
	ay_f = LPF(ay_f, AY, 0.4f);
	az_f = LPF(az_f, AZ, 0.4f);
	
	usart_s [9]  =ax_f;
	usart_s [10] =ay_f;
	
	

	float ax =ax_f - Accel_Offset_X ,  ay = ay_f - Accel_Offset_Y,  az = az_f;
	float gx = ((gyro_raw_x = ( GX -Gyro_Offset_X)/ 16.4f))*0.0174533f,  
		  gy = ((gyro_raw_y =(GY -Gyro_Offset_Y )/ 16.4f))*0.0174533f,
		  gz = ((gyro_raw_z =(GZ -Gyro_Offset_Z )/ 16.4f))*0.0174533f;
	
	gx_f = LPF(gx_f, gyro_raw_x, 0.9f);
    gy_f = LPF(gy_f, gyro_raw_y, 0.9f);
    gz_f = LPF(gz_f, gyro_raw_z, 0.9f);
	
	G->X = gx_f;
	G->Y = gy_f;
	G->Z = gz_f;
	
	float vx , vy , vz;
	float ex , ey , ez;
	float norm;
	
	
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if((ax*ax + ay*ay + az*az) < 0.0001f)
	{
		return;
	}
	
	
	//加速度计
	norm  = invSqrt(ax*ax+ay*ay+az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	
	//陀螺仪
	vx = 2*(q1q3 - q0q2);												
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	
	//叉积
	ex = (ay*vz - az*vy);                    
	ey = (az*vx - ax*vz); 
	ez = (ax*vy - ay*vx);
	
	//Ki
	exint = exint + ex * ki;								 
	eyint = eyint + ey * ki;
	ezint = ezint + ez * ki;
	
	
	//Kp
	gx = gx + kp*ex + exint;					   		  	
	gy = gy + kp*ey + eyint;
	gz = gz + kp*ez + ezint;
	
	
	//四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halft;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halft;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halft;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halft;
	
	//四元数向量
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;  
	q3 = q3 * norm;
	
	//转换欧拉角
	
	euler->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
	euler->roll = -asin(2.f * (q1q3 - q0q2))* 57.3f;                            // roll 
	euler->pitch = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ; // pitch

}



















