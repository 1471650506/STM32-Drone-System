#include "stm32f10x.h"                  // Device header
#include "KEY.h"
#include "oled.h"
#include "NRF24L01.h"
#include "Delay.h"

extern uint8_t KEY_V;
extern uint8_t NRF24L01_SendByte[32];
extern uint8_t NRF24L01_ReceiveByte[32];

float oled_menu_structuer[5] = {0,0,0,0,0x0f};

uint8_t menu_s ;



void oled_xs(void)
{
	
	
	OLED_ShowString(25,1,"Link_null",OLED_8X16);
	
	OLED_ShowString(1,16,"飞Power_null%",OLED_8X16);
	
	OLED_ShowString(1,32,"控Power_null%",OLED_8X16);
	
	OLED_ShowString(1,48,"油门_null",OLED_8X16);
	
	
	
	OLED_Update();

	
	
	
}




uint8_t OLED_MENU_START(void)
{
	uint8_t k;
	uint8_t KEY_F = 0;
	OLED_ShowString(1,1,"连接飞行器",OLED_8X16);
	OLED_ShowString(32,16,"Yes",OLED_8X16);
	OLED_ShowString(32,32,"NO",OLED_8X16);
	OLED_Update();
	
	
	while (1)
	{
		switch (KEY_GET())
		{
			case 1:
			{
				if (KEY_F == 2)
				{
					OLED_Clear();
					oled_xs();
					return 1;	
				}
				if (KEY_F ==1)
				{
					OLED_Clear();
					oled_xs();
					return 0;
				}
				
			}break;
			case 2:
			{
				OLED_Clear();
				return 0;
			}break;
			case 3:
			{
				if (KEY_F==1) 
				{
					KEY_F = 2;
				}
				else
				{
					KEY_F = 1;
				}
			}break;
			case 4:
			{
				if (KEY_F==2) 
				{
					KEY_F = 1;
				}
				else
				{
					KEY_F = 2;
				}
			}break;
		
		}
		
		KEY_V = 0;
		
		if (KEY_F == 1)
		{
			OLED_ShowString(1,16,"  ",OLED_8X16);
			OLED_ShowString(1,32,"->",OLED_8X16);
			OLED_Update();
			
		}
		if (KEY_F == 2)
		{
			OLED_ShowString(1,32,"  ",OLED_8X16);
			OLED_ShowString(1,16,"->",OLED_8X16);
			OLED_Update();
			
		}
		
		
			
		
	}	
}

uint8_t OLED_MENU_STOP(void)
{
	uint8_t KEY_F= 0;
	OLED_ShowString(1,1,"关闭飞行器",OLED_8X16);
	OLED_ShowString(16,16,"Yes",OLED_8X16);
	OLED_ShowString(16,32,"NO",OLED_8X16);
	OLED_Update();
	

	
	while (1)
	{
			switch (KEY_GET())
		{
			case 1:
			{
				if (KEY_F == 2)
				{
					OLED_Clear();
					oled_xs();
					return 1;	
				}
				if (KEY_F ==1)
				{
					OLED_Clear();
					oled_xs();
					return 0;
				}
				
			}break;
			case 2:
			{
				OLED_Clear();
				return 0;
			}break;
			case 3:
			{
				if (KEY_F == 1) 
				{
					KEY_F = 2;
				}
				else
				{
					KEY_F = 1;
				}
			}break;
			case 4:
			{
				if (KEY_F == 2) 
				{
					KEY_F = 1;
				}
				else
				{
					KEY_F = 2;
				}
			}break;
		
		}
		
		KEY_V = 0;
		
		if (KEY_F == 1)
		{
			OLED_ShowString(1,16,"  ",OLED_8X16);
			OLED_ShowString(1,32,"->",OLED_8X16);
			OLED_Update();
			
		}
		if (KEY_F == 2)
		{
			OLED_ShowString(1,32,"  ",OLED_8X16);
			OLED_ShowString(1,16,"->",OLED_8X16);
			OLED_Update();
			
		}
		
		
		
	}
}









void OLED_MENU(void)
{
	uint8_t s,T;
	
	
	if(KEY_V == 5)
	{
		OLED_Clear();
		s = OLED_MENU_START();
		
		if  (s ==1)
		{
			menu_s = s;
			NRF24L01_SendByte[10] = 0X0A;
			s=0;
		}
	}
	
		if(KEY_V == 6)
	{
		OLED_Clear();
		T = OLED_MENU_STOP();
		
		if  (T ==1)
		{
			menu_s = ~T;
			NRF24L01_SendByte[10] = 0X0F;
			
			NRF24L01_Send();
			
//			Delay_ms(100);
			
			T=0;
		}
	}
	
	
	
	KEY_V = 0;
}



void menu_structuer(void)
{
	if (oled_menu_structuer[0] == 2)
	{
		OLED_ShowString(25,1,"Link_Fall",OLED_8X16);
		OLED_Update();
	}
	else if (oled_menu_structuer[0] == 0X0F)
	{
		OLED_ShowString(25,1,"Link_Pass",OLED_8X16);
		OLED_Update();
	}
	
	
	if (oled_menu_structuer[1] != 0)
	{
		OLED_ShowString(1,16,"飞Power_  %    ",OLED_8X16);
		OLED_ShowFloatNum(66,16,oled_menu_structuer[1],1,2,OLED_8X16);
		OLED_Update();
	}
	
	if (oled_menu_structuer[2] != 0)
	{
		OLED_ShowString(1,32,"控Power_  %    ",OLED_8X16);
		OLED_ShowNum(66,32,oled_menu_structuer[2],2,OLED_8X16);
		OLED_Update();
	}
	
	if (oled_menu_structuer[3] != 0)
	{
		OLED_ShowString(1,48,"油门_      ",OLED_8X16);
		OLED_ShowNum(40,48,oled_menu_structuer[3],4,OLED_8X16);
		OLED_Update();
	}
//	if (oled_menu_structuer[4] != 0)
//	{
		OLED_ShowString(80,48,"R:",OLED_8X16);
		OLED_ShowNum(95,48,oled_menu_structuer[4],3,OLED_8X16);
		OLED_Update();
//	}
	
	
	
}







