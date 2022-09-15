#include "led.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"

u8 TIM2CH1_CAPTURE_STA = 0; // ͨ��2���벶��״̬
u16 TIM2CH1_CAPTURE_UPVAL;
u16 TIM2CH1_CAPTURE_DOWNVAL;

u8 TIM2CH2_CAPTURE_STA = 0; // ͨ��2���벶��״̬
u16 TIM2CH2_CAPTURE_UPVAL;
u16 TIM2CH2_CAPTURE_DOWNVAL;

u8 TIM2CH3_CAPTURE_STA = 0; // ͨ��3���벶��״̬
u16 TIM2CH3_CAPTURE_UPVAL;
u16 TIM2CH3_CAPTURE_DOWNVAL;

u8 TIM2CH4_CAPTURE_STA = 0; // ͨ��4���벶��״̬
u16 TIM2CH4_CAPTURE_UPVAL;
u16 TIM2CH4_CAPTURE_DOWNVAL;

u32 uptimech1 = 0; // ͨ��2����ߵ�ƽ��ʱ��
u32 uptimech2 = 0; // ͨ��2����ߵ�ƽ��ʱ��
u32 uptimech3 = 0; // ͨ��3����ߵ�ƽ��ʱ��
u32 uptimech4 = 0; // ͨ��4����ߵ�ƽ��ʱ��
u32 ahehe = 0; // �������벶���жϵĴ���
u8 direction = 0;

TIM_ICInitTypeDef TIM2_ICInitStructure;
void TIM2_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //ʹ��TIM2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // PA123���֮ǰ������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // PA123��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3); // PA123����
	
	//��ʼ����ʱ��2 TIM2
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨��ʱ���Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //��ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM2���벶����� ͨ��2
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //ѡ��CH2
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1ӳ�䵽TI1,�Դ�����234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //���������Ƶ������Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	//��ʼ��TIM2���벶����� ͨ��3
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3; //ѡ��CH3
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1ӳ�䵽TI1,�Դ�����234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //���������Ƶ������Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	//��ʼ��TIM2���벶����� ͨ��4
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //ѡ��CH4
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1ӳ�䵽TI1,�Դ�����234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //���������Ƶ������Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
    
    //��ʼ��TIM2���벶����� ͨ��4
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //ѡ��CH4
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1ӳ�䵽TI1,�Դ�����234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //���������Ƶ������Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	//�жϷ����ʼ	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ� = 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // �����ȼ� = 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure); // ��ʼ������NVIC�Ĵ���
	
	//TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE); //��������ж�,�����ж�
	TIM_ITConfig(TIM2,TIM_IT_CC4|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC1,ENABLE); // ����������ж�
	TIM_Cmd(TIM2,ENABLE); // ʹ�ܶ�ʱ��2
	
//	GPIO_InitTypeDef GPIO_InitStructure;	//????GPIO?????
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//????????????
//	TIM_ICInitTypeDef TIM_ICInitStructure;	//???????????????

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//??GPIOA??
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//?????2

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;		//??PA0->TIM2_CH1,PA1->TIM2_CH2
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//??50MHz??
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//?????????
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	TIM_TimeBaseStructure.TIM_Period = 0xffff;	//??????	
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;	//?????
//	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1;	// ??????????????
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//????
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//??????2

//	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 6;
//	TIM_ICInit(TIM2, &TIM_ICInitStructure);

//	//TIM_SetCounter(TIM2, 0x7fff);

//	//  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		//?????2?????
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
//	
//	TIM2->CNT = 0;

//	//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		//?????2??
//	TIM_Cmd(TIM2, ENABLE);  //?????,????

}
void time2_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	//????GPIO?????
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//????????????
	TIM_ICInitTypeDef TIM_ICInitStructure;	//???????????????

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//??GPIOA??
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//?????2

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;		//??PA0->TIM2_CH1,PA1->TIM2_CH2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//??50MHz??
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//?????????
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = 0xffff;	//??????	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	//?????
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	// ??????????????
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//????
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//??????2

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_SetCounter(TIM2, 0x7fff);

	//  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		//?????2?????
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		//?????2??
	TIM_Cmd(TIM2, ENABLE);  //?????,????
}
void TIM2_IRQHandler(void)
{
    	//TIM2���벶�� ͨ��2
	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!= RESET) // ͨ��2��⵽������
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC1); // ���CH2�жϱ�־λ
		if(TIM2CH1_CAPTURE_STA & 0X40) // ֮ǰ�Ѿ���⵽������,�˴μ�⵽һ���½���
		{
			TIM2CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM2); // ��¼�½��ط�����ʱ��
			if(TIM2CH1_CAPTURE_DOWNVAL < TIM2CH1_CAPTURE_UPVAL) uptimech1 = 0xffff-TIM2CH1_CAPTURE_UPVAL + TIM2CH1_CAPTURE_DOWNVAL;
				// ��TIM2��ʱ������ˣ���DOWNVAL < UPVAL������65535 - UP + DOWN = �������ص��½��صļ��� 
			else uptimech1 = TIM2CH1_CAPTURE_DOWNVAL - TIM2CH1_CAPTURE_UPVAL; // ��δ������������ص��½��صļ���= DOWN - UP
			TIM2CH1_CAPTURE_STA = 0; // �Ѿ���ɱ��β��������־λ
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // ���֮ǰû�м�⵽�����أ�����ǵ�һ�μ�⵽�����أ�
		{
			TIM2CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM2);
			TIM2CH1_CAPTURE_STA |= 0X40; // ��6λ��λ�������Ѿ���⵽һ��������
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling); // ����CH2��ʼ�ȴ��½���
		}	
	}
    
	//TIM2���벶�� ͨ��2
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)!= RESET) // ͨ��2��⵽������
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC2); // ���CH2�жϱ�־λ
		if(TIM2CH2_CAPTURE_STA & 0X40) // ֮ǰ�Ѿ���⵽������,�˴μ�⵽һ���½���
		{
			TIM2CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM2); // ��¼�½��ط�����ʱ��
			if(TIM2CH2_CAPTURE_DOWNVAL < TIM2CH2_CAPTURE_UPVAL) 
				uptimech2 = 0xffff-TIM2CH2_CAPTURE_UPVAL + TIM2CH2_CAPTURE_DOWNVAL;
				// ��TIM2��ʱ������ˣ���DOWNVAL < UPVAL������65535 - UP + DOWN = �������ص��½��صļ��� 
			else 
				uptimech2 = TIM2CH2_CAPTURE_DOWNVAL - TIM2CH2_CAPTURE_UPVAL; // ��δ������������ص��½��صļ���= DOWN - UP
			TIM2CH2_CAPTURE_STA = 0; // �Ѿ���ɱ��β��������־λ
			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // ���֮ǰû�м�⵽�����أ�����ǵ�һ�μ�⵽�����أ�
		{
			TIM2CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM2);
			TIM2CH2_CAPTURE_STA |= 0X40; // ��6λ��λ�������Ѿ���⵽һ��������
			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling); // ����CH2��ʼ�ȴ��½���
		}	
	}

	//TIM2���벶�� ͨ��3
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3)!= RESET) // ͨ��3��⵽������
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC3); // ���CH3�жϱ�־λ
		if(TIM2CH3_CAPTURE_STA & 0X40) // ֮ǰ�Ѿ���⵽������,�˴μ�⵽һ���½���
		{
			TIM2CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2); // ��¼�½��ط�����ʱ��
			if(TIM2CH3_CAPTURE_DOWNVAL < TIM2CH3_CAPTURE_UPVAL) 
				uptimech3 = 0xffff-TIM2CH3_CAPTURE_UPVAL + TIM2CH3_CAPTURE_DOWNVAL;
				// ��TIM2��ʱ������ˣ���DOWNVAL < UPVAL������65535 - UP + DOWN = �������ص��½��صļ��� 
			else 
				uptimech3 = TIM2CH3_CAPTURE_DOWNVAL - TIM2CH3_CAPTURE_UPVAL; // ��δ������������ص��½��صļ���= DOWN - UP
			TIM2CH3_CAPTURE_STA = 0; // �Ѿ���ɱ��β��������־λ
			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // ���֮ǰû�м�⵽�����أ�����ǵ�һ�μ�⵽�����أ�
		{
			TIM2CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM2);
			TIM2CH3_CAPTURE_STA |= 0X40; // ��6λ��λ�������Ѿ���⵽һ��������
			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling); // ����CH3��ʼ�ȴ��½���
		}	
	}
	
	//TIM2���벶�� ͨ��4
	if(TIM_GetITStatus(TIM2,TIM_IT_CC4)!= RESET) // ͨ��4��⵽������
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC4); // ���CH4�жϱ�־λ
		if(TIM2CH4_CAPTURE_STA & 0X40) // ֮ǰ�Ѿ���⵽������,�˴μ�⵽һ���½���
		{
			TIM2CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2); // ��¼�½��ط�����ʱ��
			if(TIM2CH4_CAPTURE_DOWNVAL < TIM2CH4_CAPTURE_UPVAL) uptimech4 = 0xffff-TIM2CH4_CAPTURE_UPVAL + TIM2CH4_CAPTURE_DOWNVAL;
				// ��TIM2��ʱ������ˣ���DOWNVAL < UPVAL������65535 - UP + DOWN = �������ص��½��صļ��� 
			else uptimech4 = TIM2CH4_CAPTURE_DOWNVAL - TIM2CH4_CAPTURE_UPVAL; // ��δ������������ص��½��صļ���= DOWN - UP
			TIM2CH4_CAPTURE_STA = 0; // �Ѿ���ɱ��β��������־λ
			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // ���֮ǰû�м�⵽�����أ�����ǵ�һ�μ�⵽�����أ�
		{
			TIM2CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM2);
			TIM2CH4_CAPTURE_STA |= 0X40; // ��6λ��λ�������Ѿ���⵽һ��������
			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling); // ����CH4��ʼ�ȴ��½���
		}	
	}
	if((TIM2CH3_CAPTURE_DOWNVAL<TIM2CH2_CAPTURE_DOWNVAL && TIM2CH3_CAPTURE_UPVAL<TIM2CH2_CAPTURE_UPVAL) 
		||(TIM2CH3_CAPTURE_UPVAL<TIM2CH2_CAPTURE_UPVAL 
	&& TIM2CH3_CAPTURE_DOWNVAL<TIM2CH2_CAPTURE_DOWNVAL + 0xffff 
	&& TIM2CH2_CAPTURE_DOWNVAL < TIM2CH2_CAPTURE_UPVAL))
	    direction = 1;
	else
		  direction = 0;
	ahehe++;
		
}

