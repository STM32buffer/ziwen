#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define BITBAND_REG(Reg,Bit) (*((uint32_t volatile*)(0x42000000u + (32u*((uint32_t)&(Reg) - (uint32_t)0x40000000u)) + (4u*((uint32_t)(Bit))))))  	
#define BLUTTOOTH_NUM 100


typedef struct Bluetooth_Queue {
 int head;//???
 int tail;//???
 uint8_t key[BLUTTOOTH_NUM];//??
 int length;//????-1 ??
}Bluetooth_Queue;

void Bluetooth_enter_Queue(Bluetooth_Queue * pq, uint8_t x);
uint8_t Bluetootth_exit_Queue(Bluetooth_Queue * pq);
void Bluetoot_Receive_isr(uint8_t temp);
void Bluetooth_Connect_State(void);
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;     
extern int  USART_state;   //����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
extern void uart_init(u32 bound);
void Bluetoot_Receive_isr1(uint8_t temp);
void USART1_DataSend(uint8_t data);
void UART_SendBytes(const uint8_t *dt, uint32_t n);
#endif


