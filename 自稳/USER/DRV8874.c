#include "sys.h"
#include "delay.h"
#include "usart.h"  
#include "DRV8874.h"
#include "pwm.h"


void DRV8874_Init()
{
	nSLEEP_set(0);
	//TIM4_PWM_Init(299, 719);
	TIM4_PWM_Init(299, 719);//72000000
	//TIM3_PWM_Init(299, 719);
	
	
	//TIM4_PWM_Init(100, 6);//72000000
  TIM3_PWM_Init(299, 2);
	
	
	//TIM4_PWM_Init(999, 23);
	//TIM3_PWM_Init(999, 23);
	
	//TIM_SetCompare1(TIM4, 300);
	//TIM_SetCompare4(TIM4, 300);
	TIM_SetCompare4(TIM3, 300);
	TIM_SetCompare3(TIM3, 300);
	//TIM_SetCompare1(TIM4, 25);
	delay_ms(1000);
	nSLEEP_set(1);
}
void nSLEEP_set(uint8_t state)
{
	if(state == 1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_4);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	}
	
}
void EN_set(uint8_t state)
{
	if(state == 1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
}
void PH_set(uint8_t state)
{
	if(state == 0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	}
}
//void nFAULT_set(uint8_t state)
//{
//	if(state == 1)
//	{
//		GPIO_SetBits(GPIOB,GPIO_Pin_6);
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOB,GPIO_Pin_6);
//	}
//}
/*state ¿ØÖÆ¿ª¹Ø
direction ¿ØÖÆ·½Ïò
speed¿ØÖÆËÙ¶
brakeÈ*/
void motor_control(uint8_t state,uint8_t direction,int speed)
{
		nSLEEP_set(state);
		PH_set(direction);
		TIM_SetCompare3(TIM3, 300-speed);
		TIM_SetCompare4(TIM3, 300-speed);
}
