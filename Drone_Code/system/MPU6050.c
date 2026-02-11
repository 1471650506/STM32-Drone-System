#include <stm32f4xx.h>
#include <math.h>
#include "My_IIC.h"
#include "MPU6050_Reg.h"
#include "MAIN.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU6050.h"
#include <stdlib.h>
#include "Delay.h"




#define MPU6050_ADDRESS		0xD0

#define DEFAULT_MPU_HZ         (100)            
#define q30                    1073741824.0f 

// 在 main 函数外定义滤波变量
float last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
// 滤波系数，0.0-1.0，越小滤波越强但延迟越大。0.5 是个折中值
float filter_alpha = 0.4f;
float last_gy = 0;

static signed char gyro_orientation[9] = { 1,  0,  0,
                                           0,  1,  0,
                                           0,  0,  1};
//float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];

float Gyro_Offset_X = 0;
float Gyro_Offset_Y = 0;
float Gyro_Offset_Z = 0;

float Accel_Offset_X = 0;
float Accel_Offset_Y = 0;


void MPU6050_W(uint8_t RegAddress,uint8_t Data)
{
	My_IIC_Start();
	My_IIC_SendByte(MPU6050_ADDRESS);
	My_IIC_ReceiveACK();
	My_IIC_SendByte(RegAddress);
	My_IIC_ReceiveACK();
	My_IIC_SendByte(Data);
	My_IIC_ReceiveACK();
	My_IIC_Stop();
	
}


uint8_t MPU6050_R(uint8_t RegAddress)
{
	uint8_t Data;
	
	My_IIC_Start();
	My_IIC_SendByte(MPU6050_ADDRESS);
	My_IIC_ReceiveACK();
	My_IIC_SendByte(RegAddress);
	My_IIC_ReceiveACK();
	
	My_IIC_Start();
	My_IIC_SendByte(MPU6050_ADDRESS | 0X01);
	My_IIC_ReceiveACK();
	Data = My_IIC_ReceiveByte();
	My_IIC_SendACK(1);
	My_IIC_Stop();
	
	return Data;
}


uint8_t MPU6050_Write_Len(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf)
{
    My_IIC_Start();
	My_IIC_SendByte((Addr << 1) | 0x00);    
	if(My_IIC_ReceiveACK())    
	{
		My_IIC_Stop();
		return 1;
	}
    My_IIC_SendByte(Reg);	  
    if(My_IIC_ReceiveACK())    
	{
		My_IIC_Stop();
		return 1;
	}
	while(Len--)
	{
		My_IIC_SendByte(*Buf++);    
		if(My_IIC_ReceiveACK())    
		{
			My_IIC_Stop();
			return 1;
		}
	}
    My_IIC_Stop();
	return 0;
} 


uint8_t MPU6050_Read_Len(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf)
{
    My_IIC_Start();
	My_IIC_SendByte((Addr << 1) | 0x00);    
	if(My_IIC_ReceiveACK())    
	{
		My_IIC_Stop();
		return 1;
	}
    My_IIC_SendByte(Reg);	  
    if(My_IIC_ReceiveACK())    
	{
		My_IIC_Stop();
		return 1;
	}	
	My_IIC_Start();
	My_IIC_SendByte((Addr << 1) | 0x01);    
	if(My_IIC_ReceiveACK())    
	{
		My_IIC_Stop();
		return 1;
	}
	while(Len--)
	{
		*Buf++ = My_IIC_ReceiveByte();    
		if(Len)  {My_IIC_SendACK(0);}     
		else     {My_IIC_SendACK(1);}     
	}
	My_IIC_Stop();
	return 0;
}














void MPU6050_Init(void)
{
	My_IIC_Init();
	
	MPU6050_W(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_W(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_W(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_W(MPU6050_CONFIG, 0x03);
	MPU6050_W(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_W(MPU6050_ACCEL_CONFIG, 0x18);
	
}


void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
	int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH,DataL;
	
	DataH = MPU6050_R(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_R(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_R(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_R(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_R(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_R(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	
	
	
	
	DataH = MPU6050_R(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_R(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_R(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_R(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_R(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_R(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;

}




void MPU6050_DMPInit(void)
{
	uint8_t res = 0;
    
	My_IIC_Init();
    res = mpu_init();
    if(!res)
    {
//        Serial_Printf("mpu initialization complete ......\r\n");
        
		//设置所需要的传感器
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//        if(!res)
//            Serial_Printf("mpu_set_sensor complete ......\r\n");
//        else
//            Serial_Printf("mpu_set_sensor come across error ......\r\n");
		
        //设置FIFO
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//        if(!res)
//            Serial_Printf("mpu_configure_fifo complete ......\r\n");
//        else
//            Serial_Printf("mpu_configure_fifo come across error ......\r\n");
        
		//设置采样率
        res = mpu_set_sample_rate(DEFAULT_MPU_HZ);
//        if(!res)
//            Serial_Printf("mpu_set_sample_rate complete ......\r\n");
//        else
//            Serial_Printf("mpu_set_sample_rate come across error ......\r\n");
        
		//加载dmp固件
        res = dmp_load_motion_driver_firmware();
//        if(!res)
//            Serial_Printf("dmp_load_motion_driver_firmware complete ......\r\n");
//        else
//            Serial_Printf("dmp_load_motion_driver_firmware come across error ......\r\n");
        
		//设置陀螺仪方向
        res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
//        if(!res)
//            Serial_Printf("dmp_set_orientation complete ......\r\n");
//        else
//            Serial_Printf("dmp_set_orientation come across error ......\r\n");
        
		//设置dmp功能
        res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |	              
              DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
              DMP_FEATURE_GYRO_CAL);
//        if(!res)
//            Serial_Printf("dmp_enable_feature complete ......\r\n");
//        else
//            Serial_Printf("dmp_enable_feature come across error ......\r\n");
        
		//设置dmp输出功率（不超过200hz）
        res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
//        if(!res)
//            Serial_Printf("dmp_set_fifo_rate complete ......\r\n");
//        else
//            Serial_Printf("dmp_set_fifo_rate come across error ......\r\n");
        
		//自检
        res = run_self_test();
//        if(!res)
//            Serial_Printf("mpu_run_self_test complete ......\r\n");
//        else
//            Serial_Printf("mpu_run_self_test come across error ......\r\n");
        
		//使能dmp
        res = mpu_set_dmp_state(1);
//        if(!res)
//            Serial_Printf("mpu_set_dmp_state complete ......\r\n");
//        else
//            Serial_Printf("mpu_set_dmp_state come across error ......\r\n");
    }
    else
    {
//        Serial_Printf("mpu initialization come across error ......\r\n");
        while(1);
    }
}








uint8_t MPU6050_ReadDMP(float *Pitch, float *Roll, float *Yaw,MPU6050_type* G)
{	
//	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))  return 1;	
//	if(sensors & INV_WXYZ_QUAT)
//	{    
//		//转为浮点数
//        q0 = quat[0] / q30;    
//        q1 = quat[1] / q30;
//        q2 = quat[2] / q30;
//        q3 = quat[3] / q30;
//        
//        *Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; 	
//        *Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
//        *Yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
//		
//		
//		
//		G->X = gyro[0]/ 16.4f;
//		G->Y = gyro[1]/ 16.4f;
//		G->Z = gyro[2]/ 16.4f;
//		
//		G->X = last_gyro_x * (1.0f - filter_alpha) + G->X * filter_alpha;
//        G->Y = last_gyro_y * (1.0f - filter_alpha) + G->Y * filter_alpha;
//        G->Z = last_gyro_z * (1.0f - filter_alpha) + G->Z * filter_alpha;		
//		
//		last_gyro_x = G->X;
//        last_gyro_y = G->Y;
//        last_gyro_z = G->Z;
//		
//		
//		
//		float current_gy = G->Y;
//		float max_change = 200.0f; // 定义单次循环允许的最大角速度变化值

//		if (abs(current_gy - last_gy) > max_change) {
//			// 如果突变超过 200，则限制其变化，防止 PID 崩溃
//			G->Y = last_gy + (current_gy > last_gy ? max_change : -max_change);
//		}
//		last_gy = G->Y;
//		
//		
//		
//    }else  return 2;
//    return 0;
}







void MPU6050_Calibrate_Offset(void)
{
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
	int32_t sum_ax = 0, sum_ay = 0;
    int16_t ax, ay, az, gx, gy, gz;
    
    
  
    for(int i = 0; i < 2000; i++)
    {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
		sum_ax += ax;
        sum_ay += ay;

        Delay_us(1000); 
    }
    
    // 计算平均值 
    Gyro_Offset_X = (float)sum_gx / 2000.0f;
    Gyro_Offset_Y = (float)sum_gy / 2000.0f;
    Gyro_Offset_Z = (float)sum_gz / 2000.0f;
    
	Accel_Offset_X = (float)sum_ax / 2000.0f;
	Accel_Offset_Y = (float)sum_ay / 2000.0f;

	
	
}














