#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif
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
////////////////////////////////////////////////////////////////////////////////// 	  
 
//=========================??????===============================
uint8_t TxBuffer[256];
uint8_t count=0;
uint8_t BTNumberSend[15] = {0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0,0};//sendTemp
uint8_t BTNumberTrue[10] = {0,0,0,0,0, 0,0,0,0,0};//vlidNum
uint16_t BTNumberTrue_16[5] = {0,0,0,0,0};
uint8_t BTNumberRece[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,};
uint8_t BTNumberErrorTrue[15] = {0,0,0,0, 0,0,0,0, 0,0};
Bluetooth_Queue Bluetooth_queue;
int Bluth_waitTime = 0;
uint8_t receFlag = 0;//??????????
uint8_t BTNumberReceCount = 0;
uint8_t receErrorTime = 0;
uint8_t headcount = 0;//֡ͷ�жϱ�־λ
uint8_t head2count = 0;//֡ͷ�ж�2
int Bluetooth_start = 0 ;
int16_t HIGH,Speed_set=0,D_yaw_control=50,D_pitch_control=50,D_roll_control=50;//�ٶ�Ƶ������
int16_t P,I,D; 
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
u8 Tx1Buffer[256];
u8 Tx1Counter=0;
u8 count1=0;
int  USART_state =0 ; 
void uart_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

		 //Usart1 NVIC ����

			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
		 //USART ��ʼ������

		USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

}

void USART1_IRQHandler(void)                	//����1�жϷ������
	{
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)
					USART_RX_STA=0;//���մ���,���¿�ʼ
				else 
					USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)
					USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}  
			Bluetoot_Receive_isr1(Res);			
     } 
		//??(????)??
	/*else if(USART_GetITStatus(USART1,USART_IT_TXE))
	{				
    USART_SendData(USART1, Tx1Buffer[Tx1Counter++]); 
    USART_ClearITPendingBit(USART1, USART_IT_TXE);
    if(Tx1Counter == count1) 
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	} 
	
	else if(USART_GetITStatus(USART1,USART_IT_ORE))
	{
		// Deal with the ORE Interrupt.
		USART_ClearITPendingBit(USART1,USART_IT_ORE);
	}*/
		
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif
} 
#endif	
void USART1_DataSend(uint8_t data)
{
    while((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������
    USART1->DR = data;
}

void UART_SendBytes(const uint8_t *dt, uint32_t n)
{
    while (n--)
    {
        BITBAND_REG(USART1->SR, 6) = 0;
        USART1->DR = *dt++;
        while (!BITBAND_REG(USART1->SR, 6));
    }
		USART1_DataSend(0x0D);
		USART1_DataSend(0x0A);

		
}
/**********************************************************
Function Name:Usart2_Send
Description:���ݰ����ͷ�����
Inputs:  unsigned char *DataToSend ,uint8_t data_num
Outputs: None
Notes:�������鶨��   
***********************************************************/
void Usart1_Send(unsigned char *DataToSend ,uint8_t data_num)
{
 uint8_t i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}

	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	}
}
/**********************************************************
Function Name:Bluetoot_Receive_isr
Description:�޸ĺ��Э���������  
Inputs:  uint8_t temp
Outputs: None
Notes:ȥ��CRCУ���������λ�����ݰ�   
***********************************************************/

uint8_t BTreceCRCTemp = 0;//��żУ���м����
uint8_t BTreceCRCFlag = 0;//��żУ���־
uint8_t receive;
u8 auto_control=0;
void Bluetoot_Receive_isr1(uint8_t temp)
{
    int i;
    receive = temp;
    if(receFlag == 1)//����PID���趨�߶�
    {
        BTNumberRece[BTNumberReceCount] = temp;
        BTNumberReceCount ++;
        //����һ�����ݰ����
        if(BTNumberReceCount >= 15)
        {
            receFlag = 0;
            BTNumberReceCount = 0;
                
            BTreceCRCTemp = 0;//��żУ���м����
            BTreceCRCFlag = 0;//��żУ���־
                
            for(i = 2; i < 12; i++)
            {
                BTreceCRCTemp = BTreceCRCTemp + BTNumberRece[i]%2;
            }
            if(BTNumberRece[12] == BTreceCRCTemp && BTNumberRece[13] == 0xf0 && BTNumberRece[14] == 0xf0)
                BTreceCRCFlag = 1;
                
            if(BTreceCRCFlag == 1)//��������׼ȷ�����
            {
                auto_control = BTNumberRece[5];
                D_yaw_control =BTNumberRece[7]+BTNumberRece[6]*256;//Kd
                Speed_set=(BTNumberRece[9]+BTNumberRece[8]*256);//�趨�߶�	
                D_pitch_control=BTNumberRece[11]+BTNumberRece[10]*256;//ʵ�ʸ߶�	      
								USART_state = 1;	
            }

        }
    }
    else
    {
        //�ж�֡ͷ
        if(temp == 0xff)
        {
            headcount ++;
        }
        else
        {
            headcount = 0;
        }
        if(headcount >= 2)
        {
            headcount = 0;
            receFlag = 1;
            BTNumberRece[0] = 0xff;
            BTNumberRece[1] = 0xff;
            BTNumberReceCount = 2;
        }
    } 
}
void Bluetoot_Receive_isr(uint8_t temp)
{
    int i;
    receive = temp;
    if(receFlag == 1)//����PID���趨�߶�
    {
        BTNumberRece[BTNumberReceCount] = temp;
        BTNumberReceCount ++;
        //����һ�����ݰ����
        if(BTNumberReceCount >= 4)
        {
 
          Speed_set=(BTNumberRece[2]);//�趨����
	      
        }
    }
    else
    {
        //�ж�֡ͷ
        if(temp == 0xee || (headcount==1 && temp == 0xff))
        {
            headcount ++;
        }
        else
        {
            headcount = 0;
        }
        if(headcount >= 2)
        {
            headcount = 0;
            receFlag = 1;
            BTNumberRece[0] = 0xee;
            BTNumberRece[1] = 0xff;
            BTNumberReceCount = 2;
        }
    } 
}


/**********************************************************
Function Name:enQueue deQueue
Description:���յ�����ѹ���ջ ����ֹ���ݶ�ʧ
Inputs:  None
Outputs: None
Notes:   
***********************************************************/
void Bluetooth_enter_Queue(struct Bluetooth_Queue * pq, uint8_t x)
{
  pq -> key[pq -> tail] = x;

  if(pq -> tail >= pq -> length) 
  {
    pq -> tail = 0;
  } 
  else 
  {
    pq -> tail++;
  }
}

uint8_t Bluetooth_exit_Queue(struct Bluetooth_Queue * pq)
{
  uint8_t x;
  
  x = pq -> key[pq -> head];
  if(pq -> head >= pq -> length) {
    pq -> head = 0;
  } 
  else 
  {
    pq -> head++;
  }
  return x;
}

/**********************************************************
Function send_Bluetooth_Data
Description:�����ݴ������ȥ
Inputs:  None
Outputs: None
Notes: ��ʱû��  
***********************************************************/
void send_Bluetooth_Data(int16_t send_Bluetooth_Data[5] )
{
  uint8_t BTsendCRCTemp = 0;
  int16_t dataTemp = 0;
	int i;
  
  //???
  BTNumberSend[0] = 0xff;
  BTNumberSend[1] = 0xff;
 
  //data*20
  //1
  dataTemp = (int16_t)send_Bluetooth_Data[0];
  BTNumberSend[2] = dataTemp >> 8;
  BTNumberSend[3] = dataTemp;
  //2
  dataTemp = (int16_t)send_Bluetooth_Data[1];
  BTNumberSend[4] = dataTemp >> 8;
  BTNumberSend[5] = dataTemp;
  //3
  dataTemp = (int16_t)send_Bluetooth_Data[2];
  BTNumberSend[6] = dataTemp >> 8;
  BTNumberSend[7] = dataTemp;
  //4
  dataTemp = (int16_t)send_Bluetooth_Data[3];
  BTNumberSend[8] = dataTemp >> 8;
  BTNumberSend[9] = dataTemp;
  //5
  dataTemp = (int16_t)send_Bluetooth_Data[4];
  BTNumberSend[10] = dataTemp >> 8;
  BTNumberSend[11] = dataTemp;
  
  for(i = 2; i < 12; i++)
  {
    BTsendCRCTemp = BTsendCRCTemp + BTNumberSend[i]%2;
  }
  BTNumberSend[12] = BTsendCRCTemp;
  //???
  BTNumberSend[13] = 0xf0;
  BTNumberSend[14] = 0xf0;
	
	Usart1_Send(BTNumberSend,15);
} // handleSendData