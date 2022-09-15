#include "delay.h"
#include "sys.h"
#include "GPIO.h"

/*
IO口初始化
配置IO口




*/

void GPIO_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PF端口时钟
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 // 端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB
//	GPIO_SetBits(GPIOB,GPIO_Pin_6);					//初始化设置为0
//	

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PF端口时钟
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 // 端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB
//	GPIO_SetBits(GPIOB,GPIO_Pin_7);					//初始化设置为0
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PF端口时钟
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 // 端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB
//	GPIO_SetBits(GPIOB,GPIO_Pin_8);					//初始化设置为0
//	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//GPIO_Pin_6|
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_5);//|GPIO_Pin_6
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//GPIO_Pin_6|
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_0);//|GPIO_Pin_6
	
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PF端口时钟
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 // 端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //上拉输入
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB
	//GPIO_SetBits(GPIOB,GPIO_Pin_6);					//初始化设置为0
}