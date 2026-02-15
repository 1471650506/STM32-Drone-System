#include "stm32f10x.h"                  // Device header
#include "oled.h"
#include "ADC_user.h"
#include "DMA_user.h"
#include "NRF24L01.h"
#include "Delay.h"
#include "KEY.h"
#include "Oled_menu.h"



uint8_t A = 0;
extern uint8_t KEY_V;
extern uint16_t  ADC_V_Send[4];
extern uint16_t ADC_V[5];
extern uint8_t NRF24L01_SendByte[32];
extern uint8_t NRF24L01_ReceiveByte[32];
extern float oled_menu_structuer[5];
extern uint8_t menu_s ;



uint16_t ADC_ReceiveByte[4];

 
 uint8_t RECEIVE[4];
 
 uint8_t ADDR[5];
 
 uint8_t NRF24L01_Data[4];
 
 uint8_t TX[5]={0X11,0X22,0X33,0X44,0X55};
 
 
 
 uint8_t SEND_strcutuer;
 
 uint8_t a;
 
 typedef union {
    float f;          // 浮点视角 (存数据用)
    unsigned char b[4]; // 字节视角 (发数据用)
} FloatPacket;
 

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

void pid_set(void)
{
	//内环pid
	PID_ROL_Rate_p.f = 1.55f;
	PID_PIT_Rate_p.f = 1.55f;
	PID_YAW_Rate_p.f = -1.0f;
	
	PID_ROL_Rate_i.f = 0.03f;
	PID_PIT_Rate_i.f = 0.03f;
	PID_YAW_Rate_i.f = 0.00f;
	
	PID_ROL_Rate_d.f = 0.4f;
    PID_PIT_Rate_d.f = 0.4f;
    PID_YAW_Rate_d.f = 0;

	//外环pid
	
	PID_ROL_Angle_p.f =5.00f;
	PID_PIT_Angle_p.f = -5.00f;
	PID_YAW_Angle_p.f = 0;
	
	PID_ROL_Angle_i.f = 0;
	PID_PIT_Angle_i.f = 0;
	PID_YAW_Angle_i.f = 0;
	               
	PID_ROL_Angle_d.f = 0;
	PID_PIT_Angle_d.f = 0;
	PID_YAW_Angle_d.f = 0;
	
	//角度修正
	
//	roll .f = 0.7f;
//	pit  .f = -2.95f;
//	yaw  .f = 0;
	
}

//发送一组pid
void pid_send(FloatPacket*p,FloatPacket*i,FloatPacket*d,uint8_t id)
{
	

		NRF24L01_SendByte[12] = p->b[0];
		NRF24L01_SendByte[13] = p->b[1];
		NRF24L01_SendByte[14] = p->b[2];
		NRF24L01_SendByte[15] = p->b[3];
	
	
		NRF24L01_SendByte[16] = i->b[0];
		NRF24L01_SendByte[17] = i->b[1];
		NRF24L01_SendByte[18] = i->b[2];
		NRF24L01_SendByte[19] = i->b[3];
	
		NRF24L01_SendByte[20] = d->b[0];
		NRF24L01_SendByte[21] = d->b[1];
		NRF24L01_SendByte[22] = d->b[2];
		NRF24L01_SendByte[23] = d->b[3];

		NRF24L01_SendByte[24] = id;
		NRF24L01_SendByte[25] = 0XFF;
	
		 SEND_strcutuer = NRF24L01_Send();
	 
	
	
	
	 Delay_ms(10);
	
	
}

void pid(void)
{
	
	
	pid_set();
	
	for(int i=0; i<3; i++)
  {
	uint8_t id =1;
	  
	//内环发送
	pid_send(&PID_ROL_Rate_p,&PID_ROL_Rate_i,&PID_ROL_Rate_d,id++);
	pid_send(&PID_PIT_Rate_p,&PID_PIT_Rate_i,&PID_PIT_Rate_d,id++);
	pid_send(&PID_YAW_Rate_p,&PID_YAW_Rate_i,&PID_YAW_Rate_d,id++);
	
	//外环发送
	pid_send(&PID_ROL_Angle_p,&PID_ROL_Angle_i,&PID_ROL_Angle_d,id++);
	pid_send(&PID_PIT_Angle_p,&PID_PIT_Angle_i,&PID_PIT_Angle_d,id++);
	pid_send(&PID_YAW_Angle_p,&PID_YAW_Angle_i,&PID_YAW_Angle_d,id++);
	
	//修正发送
	pid_send(&roll,&pit,&yaw,id++);
	  
	Delay_ms(20);
	  
  }
}

 /*
 
 NRF24L01_SendByte[0]       开始位
 NRF24L01_SendByte[1]       读写控制位,0X0F表示我要读取你的本机信息
 NRF24L01_SendByte[2] 		
 NRF24L01_SendByte[3] 		
 NRF24L01_SendByte[4] 		
 NRF24L01_SendByte[5] 		
 NRF24L01_SendByte[6] 		
 NRF24L01_SendByte[7] 		
 NRF24L01_SendByte[8] 		
 NRF24L01_SendByte[9] 		
 NRF24L01_SendByte[10]		停止控制位
 NRF24L01_SendByte[11]		结束位
 
 */
 
 
 
void NRF24L01_ADC_SendByte(void)
{
	 ADC_Send();

	
	 NRF24L01_SendByte[0] = 0XAA;
//	 NRF24L01_SendByte[1] = 0;
	 NRF24L01_SendByte[2] = ADC_V_Send[0]>>8;
	 NRF24L01_SendByte[3] = ADC_V_Send[0];
	 NRF24L01_SendByte[4] = ADC_V_Send[1]>>8;
	 NRF24L01_SendByte[5] = ADC_V_Send[1];
	 NRF24L01_SendByte[6] = ADC_V_Send[2]>>8;
	 NRF24L01_SendByte[7] = ADC_V_Send[2];
	 NRF24L01_SendByte[8] = ADC_V_Send[3]>>8;
	 NRF24L01_SendByte[9] = ADC_V_Send[3];
//	 NRF24L01_SendByte[10] = ADC_V[4]>>8;
	 NRF24L01_SendByte[11] = 0XFF;
	 
	NRF24L01_TxMode();
	 
	 SEND_strcutuer = NRF24L01_Send();
	 
	NRF24L01_RxMode();
	
	
	 Delay_us(100);
	 
}
 
 
  uint8_t NRF24L01_ADC_ReceiveByte(void)
{
	float p =0;
	uint8_t a = NRF24L01_Receive();
	
	if (a == 0)
	{
		
		
	uint8_t Battery = Battery_GetPercent(Battery_GetVoltage());
	
	
	p = ((float)NRF24L01_ReceiveByte[6]/100)+3;
	
	
	if(NRF24L01_ReceiveByte[6]>130)
	{
		
		p = 0;
		
	}
	
	oled_menu_structuer[0] = NRF24L01_ReceiveByte[2];  //连接状况,遥控板发给无人机,再由无人机给出返回值
	oled_menu_structuer[1] = p;  //数组[3]为无人机电池百分比显示,[6]为具体电压显示
	oled_menu_structuer[2] = Battery;					//本机电池
//	oled_menu_structuer[3] = NRF24L01_ReceiveByte[4];  //这个是bmp读出的高度,但是bmp未安装
	oled_menu_structuer[4] = NRF24L01_ReceiveByte[5];  //这个是收发计数,由无人机发出
	
	
	return 0;
	
	
	}
	return 1;
}
 
  uint8_t NRF24L01_ADC_ReceiveByte_Link(void)
{
	uint8_t a = NRF24L01_Receive_Link();
	
	if (a ==0)
	{
	
	uint8_t Battery  = Battery_GetPercent(Battery_GetVoltage());
	oled_menu_structuer[0] = NRF24L01_ReceiveByte[2];
	oled_menu_structuer[1] = NRF24L01_ReceiveByte[6];  //数组[3]为无人机电池百分比显示,[6]为具体电压显示
	oled_menu_structuer[2] = Battery;					//本机电池
	oled_menu_structuer[4] = NRF24L01_ReceiveByte[5];  //这个是收发计数,由无人机发出
	
	return 0;
	}
	return 1;
}









void link_init(void)
{
	NRF24L01_TxMode();
	
	while (1)
	{
		OLED_MENU();
		menu_structuer();
		
	if (menu_s == 1)
		 {
			NRF24L01_SendByte[1] = 0X00;
			pid();
//			Delay_ms(100);
			NRF24L01_SendByte[1] = 0X0F;	
			NRF24L01_ADC_SendByte();
			
			
			 
			 Delay_ms(1);
			 
			 NRF24L01_RxMode();
			 
			while (1)
			{
				NRF24L01_ADC_ReceiveByte_Link () ;
			
				if (oled_menu_structuer[0] != 0)
				{
					menu_structuer();
			
					NRF24L01_SendByte[1] = 0;
					
					return ;
				}
				
				
			}				
			 
			 
			 
		 }
		 
		 
		 
		 
	}
	
	
	
	
	
	
	
}






 
int main(void)
 {
	 uint32_t menu_count = 0;
		uint32_t count = 0 ;
	 OLED_Init();
	 NRF24L01_GPIO_Init();

	 ADC_user_Init();
	 
	 NRF24L01_Init();
	 
	 KEY_Init();
	 
	 ADC_V_ResetCalibration();

	 oled_xs();
	 
	 
	 
	 menu_s = 2;
	 
	 
		NRF24L01_SendByte[0] =1;
		NRF24L01_SendByte[1] =1;
		NRF24L01_SendByte[2] =1;
		NRF24L01_SendByte[3] =1;
		
		
		
		
		link_init();
		
	 
	 while (1)
	 {
		OLED_MENU();
		
		 
		 
		
		 if (menu_s == 1)
		 {
			
				NRF24L01_ADC_SendByte();
				count ++;
			 
			 
		 }
		 else if (menu_s == 0)
			 
		 {
			 
			 if (menu_count  < 11)
			 {
			 NRF24L01_SendByte[10] = 0X0F;
			 NRF24L01_ADC_SendByte();
			 menu_count++;	 
				 
			 }
			 
			 
		 }
		 
		 if (menu_count > 10)
		 {
			 
			 
			 NRF24L01_ADC_ReceiveByte () ;
			 
		 }
//		menu_structuer();
		 
		if (count > 50 )
		{
			NRF24L01_SendByte[1] = 0X0F;
			
			NRF24L01_ADC_SendByte();
			
			
			
			uint16_t wait_timeout = 0;
			while(wait_timeout < 500) // 等待约5ms (具体时间看你的Delay实现)
			{
				// 如果接收成功(返回0表示成功，看你的Receive函数逻辑)
				if(NRF24L01_ADC_ReceiveByte() == 0) 
				{
					menu_structuer(); // 处理数据
					break; // 收到数据就退出等待
				}
				Delay_us(10); 
				wait_timeout++;
			}
			
			
			
			menu_structuer();
			
			NRF24L01_SendByte[1] = 0;
			
			count  =0;
			
		}
		 
		 
		
		
		
		
		
		
		
		
		
		
		menu_structuer();
		
		
		
		Delay_ms(10);
	 }
	 
	 
	 

 }
 
 