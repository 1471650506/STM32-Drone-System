#include "stm32f4xx.h"                  // Device header
#include "My_IIC.h"
#include "Oled.h"
#include "Delay.h"
#include "MPU6050.h"
#include "MYI2C.h"
#include "imu.h"
#include "MY_USART1.h"
#include "NRF24L01.h"
#include "stdio.h"
#include "PID.h"
#include "ADC.h"
#include "Motor.h"
#include "Main.h"

extern volatile uint8_t pid_execute_flag;
extern uint8_t NRF24L01_SendByte[32];
extern uint8_t NRF24L01_ReceiveByte[32];

volatile uint8_t rest_CONTROL = 0;

volatile uint8_t count = 0;

uint8_t Ack_Status = 0x0F;





// typedef union {
//    float f;          // 浮点视角 (存数据用)
//    unsigned char b[4]; // 字节视角 (发数据用)
//} FloatPacket;

//内环pid
FloatPacket PID_ROL_Rate_p,PID_ROL_Rate_i,PID_ROL_Rate_d;
FloatPacket PID_PIT_Rate_p,PID_PIT_Rate_i,PID_PIT_Rate_d;
FloatPacket PID_YAW_Rate_p,PID_YAW_Rate_i,PID_YAW_Rate_d;

//外环pid
FloatPacket PID_ROL_Angle_p,PID_ROL_Angle_i,PID_ROL_Angle_d;
FloatPacket PID_PIT_Angle_p,PID_PIT_Angle_i,PID_PIT_Angle_d;
FloatPacket PID_YAW_Angle_p,PID_YAW_Angle_i,PID_YAW_Angle_d;

//角度修正
FloatPacket roll,pit,yaw;



NRF_Receive Receive;


MPU6050_type G;



int fputc(int ch, FILE *f)
{
    // 等待发送缓冲区为空
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    
    // 发送字符
    USART_SendData(USART1, (uint8_t)ch);
    
    return ch;
}



EulerAngles_t attitude;

uint8_t rxbuf[32];
uint8_t txbuf[32];


void NRF24L01_ADC_SendByte(void)
{
	float vbat;
	uint8_t p ,p1,p2;
	
	NRF24L01_TxMode();
	
	
	vbat = Battery_GetVoltage();
	
	p1 = (uint8_t)((vbat-3)*100);
	
	
	
	
	
	p = Battery_GetPercent(vbat);
	
	
	
	 NRF24L01_SendByte[0] = 0XAA;
//	 NRF24L01_SendByte[1] = 0;
	 NRF24L01_SendByte[2] = 0X0F;//连接状态
	 NRF24L01_SendByte[3] = p;//电池
	 NRF24L01_SendByte[4] = 0;//高度bmp280
	 NRF24L01_SendByte[5] = Ack_Status;
	 NRF24L01_SendByte[6] = p1;
//	 NRF24L01_SendByte[7] = ADC_V_Send[2];
//	 NRF24L01_SendByte[8] = ADC_V_Send[3]>>8;
//	 NRF24L01_SendByte[9] = ADC_V_Send[3];
//	 NRF24L01_SendByte[10] = ADC_V[4]>>8;
	 NRF24L01_SendByte[11] = 0XFF;
	 
	 
	 
	 NRF24L01_SendByte[12] = usart_s[0];
	 NRF24L01_SendByte[13] = usart_s[1];
	 NRF24L01_SendByte[14] = usart_s[2];
	 NRF24L01_SendByte[15] = usart_s[3];
	 NRF24L01_SendByte[16] = usart_s[4];
	 NRF24L01_SendByte[17] = usart_s[5];
	 NRF24L01_SendByte[18] = usart_s[6];
	 NRF24L01_SendByte[19] = usart_s[7];
	 NRF24L01_SendByte[20] = usart_s[8];
	
	 
	
	NRF24L01_Send();
	
	Delay_ms(2);

	NRF24L01_RxMode();
}
 
 
uint8_t NRF24L01_ADC_ReceiveByte(void)
{
	int32_t P = 0,Y = 0;
	
	
	
	
	if (NRF24L01_Receive() != 0) 
    {
        return 0; // 没收到数据，直接返回
    }
	 
	
	Receive.START =	 	NRF24L01_ReceiveByte[0] ;
	Receive.CONTROL =	NRF24L01_ReceiveByte[1] ;
	P =              	NRF24L01_ReceiveByte[2]<<8 |NRF24L01_ReceiveByte[3] ;
						
	Receive.yaw =		NRF24L01_ReceiveByte[4]<<8 |NRF24L01_ReceiveByte[5] ;
						
	Receive.pitch = 	NRF24L01_ReceiveByte[6]<<8 |NRF24L01_ReceiveByte[7] ;
						
	Receive.roll = 		NRF24L01_ReceiveByte[8]<<8 |NRF24L01_ReceiveByte[9] ;
						
	Receive.off = 		NRF24L01_ReceiveByte[10];
	Receive.STOP =		NRF24L01_ReceiveByte[11];
	
	
	if (P<520)
	{
		P = 520;
		
	}
	
	
	Y = P-520;
	
	Y = (P - 520) * 900 / 480;

	if(Y > 900)
	{
		Y = 900;
	}
	
	
	
	Receive.POWER = Y;
	
	
	NRF24L01_SendByte[7] = Y;
	
	
	return 1;
	
}

void pid_receive(FloatPacket*p,FloatPacket*i,FloatPacket*d,uint8_t id)
{
	
	
	p->b[0]   = NRF24L01_ReceiveByte[12] ;
	p->b[1]   = NRF24L01_ReceiveByte[13] ;
	p->b[2]   = NRF24L01_ReceiveByte[14] ;
	p->b[3]   = NRF24L01_ReceiveByte[15] ;
		      
		      
	i->b[0]   = NRF24L01_ReceiveByte[16] ;
	i->b[1]   = NRF24L01_ReceiveByte[17] ;
	i->b[2]   = NRF24L01_ReceiveByte[18] ;
	i->b[3]   = NRF24L01_ReceiveByte[19] ;
		      
	d->b[0]   = NRF24L01_ReceiveByte[20] ;
	d->b[1]   = NRF24L01_ReceiveByte[21] ;
	d->b[2]   = NRF24L01_ReceiveByte[22] ;
	d->b[3]   = NRF24L01_ReceiveByte[23] ;
		                                
	id		  = NRF24L01_ReceiveByte[24] ;
	
	
	
	
}

void START_init(void)
{
	
	uint8_t id;
	while (1)
	{
		
		NRF24L01_ADC_ReceiveByte();
		
		switch(NRF24L01_ReceiveByte[24])
		{
			case 1: pid_receive(&PID_ROL_Rate_p,&PID_ROL_Rate_i,&PID_ROL_Rate_d,id); break;
			case 2: pid_receive(&PID_PIT_Rate_p,&PID_PIT_Rate_i,&PID_PIT_Rate_d,id); break;
			case 3: pid_receive(&PID_YAW_Rate_p,&PID_YAW_Rate_i,&PID_YAW_Rate_d,id); break;
			case 4: pid_receive(&PID_ROL_Angle_p,&PID_ROL_Angle_i,&PID_ROL_Angle_d,id); break;
			case 5: pid_receive(&PID_PIT_Angle_p,&PID_PIT_Angle_i,&PID_PIT_Angle_d,id); break;
			case 6: pid_receive(&PID_YAW_Angle_p,&PID_YAW_Angle_i,&PID_YAW_Angle_d,id); break;
			case 7: pid_receive(&roll,&pit,&yaw,id); break; // 修正值
		}
		
		
		if (Receive.CONTROL == 0X0F)
        {
            // 只要 Roll 和 Pitch 的 P 值不是 0，就认为同步成功
            if (PID_ROL_Rate_p.f != 0 && PID_PIT_Rate_p.f != 0)
            {
                Ack_Status = 0x00; //成功
            }
            else
            {
                Ack_Status = 0x01; //失败
            }
		
		
		

			
			NRF24L01_TxMode();
			
			Delay_ms(100);
			
			NRF24L01_ADC_SendByte();
			
			
			Delay_ms(100);
			
			NRF24L01_RxMode();
			
			return ;
			
		}
		
		
		
	}
	
	
	
	
	
	
}








int main()
{


 	OLED_Init();
	MPU6050_Init();
	
	MPU6050_Calibrate_Offset();
	
//	MPU6050_DMPInit();
	USART1_Init(115200);
//	NRF24L01_Init();
	TIM3_PWM_Init();
	
	NRF24L01_SendByte[0]=0;
	
	NRF24L01_SendByte[1] = 0;
	count = 0;
	uint8_t a;
	
	ADC1_CH0_Init();
	
	NRF24L01_Init_();
	
	
	
	TIM4_Int_Init();
	

	
	NRF24L01_RxMode();
	
	uint16_t nrf_count = 0;

	START_init();
	
	PID_structuer_Init();
	
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64); 
    IWDG_SetReload(250); 
    IWDG_ReloadCounter();
    IWDG_Enable();
	
	
	
	while (1)
	{		
		
		
		IWDG_ReloadCounter();
		
		
		if (NRF24L01_ADC_ReceiveByte() == 1)
        {
            
            nrf_count = 0;
			
        }
		
		
		
		if (pid_execute_flag ==1)
		{
			pid_execute_flag = 0;
			
			nrf_count++;
			
			
			//未接收到数据锁死,并跳过解算
			
			if (nrf_count > 500) 
            {
                TIM_CCR(0,0,0,0); 
                Receive.off = 0X0F; 
                continue; 
				
            }
			
			
			IMU(&attitude,&G);
			
			
//			//超过45度,锁死
//			
//			if (attitude.roll > 45.0f || attitude.roll < -45.0f || 
//                attitude.pitch > 45.0f || attitude.pitch < -45.0f)
//            {
//                 TIM_CCR(0,0,0,0);
//                 Receive.off = 0X0F;
//				
//				rest_CONTROL = 1;
//				
//            }
			
			
					if ((Receive.off ==0X0A) &&  (rest_CONTROL ==0))
					{
			 
						control_pid(&attitude,&G,&Receive);
			
			 
			
					}
					else if (Receive.off ==0X0F) 
					{
						rest_CONTROL = 0;
			
						TIM_CCR(0,0,0,0);
			
						NRF24L01_ADC_SendByte();
			
					}
			

//							printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f  \r\n", usart_s[0], usart_s[1], usart_s[2], usart_s[3], usart_s[4], usart_s[5], usart_s[6], usart_s[7], usart_s[8], usart_s[9], usart_s[10], attitude.roll, attitude.pitch, attitude.yaw); //这个是用串口给vofa发送数据包

					
					
					
					
		}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		if (NRF24L01_ReceiveByte[1] ==0x0f )
		{
			count ++;
			
			
			NRF24L01_ADC_SendByte();
			
			NRF24L01_ReceiveByte[1] = 0 ;
			
			
		}
		
		
		
		
		
		
	}
	
}











