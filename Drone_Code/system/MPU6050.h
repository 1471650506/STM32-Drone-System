#ifndef __MPU6050_H
#define __MPU6050_H

#include <stm32f4xx.h>




extern float Gyro_Offset_X;
extern float Gyro_Offset_Y;
extern float Gyro_Offset_Z;
extern float Accel_Offset_X;
extern float Accel_Offset_Y;






typedef struct {
    float X;
	float Y;
	float Z;	
} MPU6050_type;




void MPU6050_W(uint8_t RegAddress,uint8_t Data);

uint8_t MPU6050_R(uint8_t RegAddress);

void MPU6050_Init(void);

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
	int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);











uint8_t MPU6050_Write_Len(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf);

uint8_t MPU6050_Read_Len(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf);


void MPU6050_DMPInit(void);

uint8_t MPU6050_ReadDMP(float *Pitch, float *Roll, float *Yaw,MPU6050_type* G);

void MPU6050_Calibrate_Offset(void);


#endif