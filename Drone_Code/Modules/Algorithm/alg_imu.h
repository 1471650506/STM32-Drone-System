#include <stm32f4xx.h>
#include "Main.h"
#include "dev_mpu6050.h"


#ifndef __IMU_H
#define __IMU_H

typedef struct {
    float roll;   // 滚转角
    float pitch;  // 俯仰角  
    float yaw;    // 偏航角
} EulerAngles_t;




void IMU(EulerAngles_t* euler,MPU6050_type* G);









#endif
