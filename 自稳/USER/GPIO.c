#include "delay.h"
#include "sys.h"
#include "GPIO.h"

/*
IO�ڳ�ʼ��
����IO��




*/

void GPIO_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PF�˿�ʱ��
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 // �˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB
//	GPIO_SetBits(GPIOB,GPIO_Pin_6);					//��ʼ������Ϊ0
//	

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PF�˿�ʱ��
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 // �˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB
//	GPIO_SetBits(GPIOB,GPIO_Pin_7);					//��ʼ������Ϊ0
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PF�˿�ʱ��
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 // �˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB
//	GPIO_SetBits(GPIOB,GPIO_Pin_8);					//��ʼ������Ϊ0
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
	
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PF�˿�ʱ��
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 // �˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //��������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB
	//GPIO_SetBits(GPIOB,GPIO_Pin_6);					//��ʼ������Ϊ0
}