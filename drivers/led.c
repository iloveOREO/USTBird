/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：led.c
 * 描述    ：LED驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "led.h"
#include "include.h"

void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_StructInit(&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED2| ANO_Pin_LED3| ANO_Pin_LED4;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
	
	
	GPIO_SetBits(ANO_GPIO_LED, ANO_Pin_LED1);		
	GPIO_SetBits(ANO_GPIO_LED, ANO_Pin_LED2);		
	GPIO_SetBits(ANO_GPIO_LED, ANO_Pin_LED3);		
	GPIO_SetBits(ANO_GPIO_LED, ANO_Pin_LED4);		
	
// 	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED1);		
// 	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED2);		
// 	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED3);		
// 	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED4);	


}

void LED_Display( u8 duty[4] ) //0~20
{
	static u8 LED_cnt[4];
	u8 i;
	
	for(i=0;i<4;i++)
	{
		if( LED_cnt[i] < 19 )
		{
			LED_cnt[i]++;
		}
		else
		{
			LED_cnt[i] = 0;
		}
			
		if( LED_cnt[i] < duty[i] )
		{
			switch(i)
			{
				case 0:	
					LED1_ON;
				break;
				case 1:	
					LED2_ON;
				break;
				case 2:	
					LED3_ON;
				break;
				case 3:	
					LED4_ON;
				break;
			}
		}
		else
		{
						switch(i)
			{
				case 0:	
					LED1_OFF;
				break;
				case 1:	
					LED2_OFF;
				break;
				case 2:	
					LED3_OFF;
				break;
				case 3:	
					LED4_OFF;
				break;
			}
		}
	}
}

#include "rc.h"
#include "ak8975.h"

extern u8 height_ctrl_mode;

u8 LED_Brightness[4] = {0,20,0,0}; //TO 20
u8 LED_status[2];  //  0:old;  1:now
void LED_Duty()
{
	static s16 led_temp;
	static u8 f;
	
	if(Mag_CALIBRATED)
	{
		LED_status[1] = 1;
	}
	else if(height_ctrl_mode==1)
	{
		LED_status[1] = 2;
	}
	else if(height_ctrl_mode==2)
	{
		LED_status[1] = 3;
	}
	else if(height_ctrl_mode==0)
	{
		LED_status[1] = 0;
	}
	
	switch(LED_status[1])
	{
		case 0:
			if(!fly_ready)
			{
				if( f )
				{
					led_temp += 4;
					if(led_temp>100) f = 0;
				}
				else
				{
					led_temp -= 4;
					if(led_temp < 0 ) f = 1;
				}
				
				LED_Brightness[1] = led_temp/10;
				LED_Brightness[2] = led_temp/10;
				LED_Brightness[3] = led_temp/10;
			}
			else
			{
				LED_Brightness[0] = 0;
				LED_Brightness[1] = 0;
				LED_Brightness[2] = 0;
				LED_Brightness[3] = 20;
			}
		break;
		case 1:
				if( f )
				{
					led_temp += 12;
					if(led_temp>100) f = 0;
				}
				else
				{
					led_temp -= 12;
					if(led_temp < 0 ) f = 1;
				}
				LED_Brightness[0] = 0;			
				LED_Brightness[1] = led_temp/10;
				LED_Brightness[2] = 0;
				LED_Brightness[3] = led_temp/10;
		break;
		case 2:
			if(!fly_ready)
			{
				if( f )
				{
					led_temp += 4;
					if(led_temp>100) f = 0;
				}
				else
				{
					led_temp -= 4;
					if(led_temp < 0 ) f = 1;
				}
				
				LED_Brightness[1] = 0;
				LED_Brightness[2] = led_temp/10;
				LED_Brightness[3] = led_temp/10;
			}
			else
			{
				LED_Brightness[0] = 0;
				LED_Brightness[1] = 0;
				LED_Brightness[2] = 12;
				LED_Brightness[3] = 12;
			}
		break;
		case 3:
			if(!fly_ready)
			{
				if( f )
				{
					led_temp += 4;
					if(led_temp>100) f = 0;
				}
				else
				{
					led_temp -= 4;
					if(led_temp < 0 ) f = 1;
				}
				
				LED_Brightness[1] = led_temp/10;
				LED_Brightness[2] = led_temp/10;
				LED_Brightness[3] = 0;
			}
			else
			{
				LED_Brightness[0] = 0;
				LED_Brightness[1] = 12;
				LED_Brightness[2] = 12;
				LED_Brightness[3] = 0;
			}
		break;
				
		default:break;

	}

}

void LED_MPU_Err(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	while(1)
	{
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(200);
		
	}
}

void LED_Mag_Err(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	while(1)
	{
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(150);
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(600);
	}
}

void LED_MS5611_Err(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	while(1)
	{
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(150);
		
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(150);
		
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(600);
	}

}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

