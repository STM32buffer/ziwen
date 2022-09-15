#include "led.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"

u8 TIM2CH1_CAPTURE_STA = 0; // 通道2输入捕获状态
u16 TIM2CH1_CAPTURE_UPVAL;
u16 TIM2CH1_CAPTURE_DOWNVAL;

u8 TIM2CH2_CAPTURE_STA = 0; // 通道2输入捕获状态
u16 TIM2CH2_CAPTURE_UPVAL;
u16 TIM2CH2_CAPTURE_DOWNVAL;

u8 TIM2CH3_CAPTURE_STA = 0; // 通道3输入捕获状态
u16 TIM2CH3_CAPTURE_UPVAL;
u16 TIM2CH3_CAPTURE_DOWNVAL;

u8 TIM2CH4_CAPTURE_STA = 0; // 通道4输入捕获状态
u16 TIM2CH4_CAPTURE_UPVAL;
u16 TIM2CH4_CAPTURE_DOWNVAL;

u32 uptimech1 = 0; // 通道2捕获高电平的时间
u32 uptimech2 = 0; // 通道2捕获高电平的时间
u32 uptimech3 = 0; // 通道3捕获高电平的时间
u32 uptimech4 = 0; // 通道4捕获高电平的时间
u32 ahehe = 0; // 进入输入捕获中断的次数
u8 direction = 0;

TIM_ICInitTypeDef TIM2_ICInitStructure;
void TIM2_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //使能TIM2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // PA123清除之前的设置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // PA123下拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3); // PA123下拉
	
	//初始化定时器2 TIM2
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计时器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //初始化TIMx的时间基数单位
	
	//初始化TIM2输入捕获参数 通道2
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //选择CH2
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1映射到TI1,以此类推234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频，不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	//初始化TIM2输入捕获参数 通道3
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3; //选择CH3
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1映射到TI1,以此类推234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频，不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	//初始化TIM2输入捕获参数 通道4
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //选择CH4
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1映射到TI1,以此类推234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频，不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
    
    //初始化TIM2输入捕获参数 通道4
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //选择CH4
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // IC1映射到TI1,以此类推234
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频，不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0X00; // IC1F = 0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	//中断分组初始	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级 = 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 子优先级 = 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure); // 初始化外设NVIC寄存器
	
	//TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE); //允许更新中断,捕获中断
	TIM_ITConfig(TIM2,TIM_IT_CC4|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC1,ENABLE); // 不允许更新中断
	TIM_Cmd(TIM2,ENABLE); // 使能定时器2
	
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
    	//TIM2输入捕获 通道2
	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!= RESET) // 通道2检测到跳变沿
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC1); // 清除CH2中断标志位
		if(TIM2CH1_CAPTURE_STA & 0X40) // 之前已经检测到上升沿,此次检测到一个下降沿
		{
			TIM2CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM2); // 记录下降沿发生的时间
			if(TIM2CH1_CAPTURE_DOWNVAL < TIM2CH1_CAPTURE_UPVAL) uptimech1 = 0xffff-TIM2CH1_CAPTURE_UPVAL + TIM2CH1_CAPTURE_DOWNVAL;
				// 若TIM2定时器溢出了，则DOWNVAL < UPVAL，则由65535 - UP + DOWN = 从上升沿到下降沿的计数 
			else uptimech1 = TIM2CH1_CAPTURE_DOWNVAL - TIM2CH1_CAPTURE_UPVAL; // 若未溢出，则上升沿到下降沿的计数= DOWN - UP
			TIM2CH1_CAPTURE_STA = 0; // 已经完成本次捕获，清除标志位
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // 如果之前没有检测到上升沿（这次是第一次检测到上升沿）
		{
			TIM2CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM2);
			TIM2CH1_CAPTURE_STA |= 0X40; // 第6位置位，代表已经检测到一次上升沿
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling); // 设置CH2开始等待下降沿
		}	
	}
    
	//TIM2输入捕获 通道2
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)!= RESET) // 通道2检测到跳变沿
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC2); // 清除CH2中断标志位
		if(TIM2CH2_CAPTURE_STA & 0X40) // 之前已经检测到上升沿,此次检测到一个下降沿
		{
			TIM2CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM2); // 记录下降沿发生的时间
			if(TIM2CH2_CAPTURE_DOWNVAL < TIM2CH2_CAPTURE_UPVAL) 
				uptimech2 = 0xffff-TIM2CH2_CAPTURE_UPVAL + TIM2CH2_CAPTURE_DOWNVAL;
				// 若TIM2定时器溢出了，则DOWNVAL < UPVAL，则由65535 - UP + DOWN = 从上升沿到下降沿的计数 
			else 
				uptimech2 = TIM2CH2_CAPTURE_DOWNVAL - TIM2CH2_CAPTURE_UPVAL; // 若未溢出，则上升沿到下降沿的计数= DOWN - UP
			TIM2CH2_CAPTURE_STA = 0; // 已经完成本次捕获，清除标志位
			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // 如果之前没有检测到上升沿（这次是第一次检测到上升沿）
		{
			TIM2CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM2);
			TIM2CH2_CAPTURE_STA |= 0X40; // 第6位置位，代表已经检测到一次上升沿
			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling); // 设置CH2开始等待下降沿
		}	
	}

	//TIM2输入捕获 通道3
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3)!= RESET) // 通道3检测到跳变沿
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC3); // 清除CH3中断标志位
		if(TIM2CH3_CAPTURE_STA & 0X40) // 之前已经检测到上升沿,此次检测到一个下降沿
		{
			TIM2CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2); // 记录下降沿发生的时间
			if(TIM2CH3_CAPTURE_DOWNVAL < TIM2CH3_CAPTURE_UPVAL) 
				uptimech3 = 0xffff-TIM2CH3_CAPTURE_UPVAL + TIM2CH3_CAPTURE_DOWNVAL;
				// 若TIM2定时器溢出了，则DOWNVAL < UPVAL，则由65535 - UP + DOWN = 从上升沿到下降沿的计数 
			else 
				uptimech3 = TIM2CH3_CAPTURE_DOWNVAL - TIM2CH3_CAPTURE_UPVAL; // 若未溢出，则上升沿到下降沿的计数= DOWN - UP
			TIM2CH3_CAPTURE_STA = 0; // 已经完成本次捕获，清除标志位
			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // 如果之前没有检测到上升沿（这次是第一次检测到上升沿）
		{
			TIM2CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM2);
			TIM2CH3_CAPTURE_STA |= 0X40; // 第6位置位，代表已经检测到一次上升沿
			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling); // 设置CH3开始等待下降沿
		}	
	}
	
	//TIM2输入捕获 通道4
	if(TIM_GetITStatus(TIM2,TIM_IT_CC4)!= RESET) // 通道4检测到跳变沿
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC4); // 清除CH4中断标志位
		if(TIM2CH4_CAPTURE_STA & 0X40) // 之前已经检测到上升沿,此次检测到一个下降沿
		{
			TIM2CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2); // 记录下降沿发生的时间
			if(TIM2CH4_CAPTURE_DOWNVAL < TIM2CH4_CAPTURE_UPVAL) uptimech4 = 0xffff-TIM2CH4_CAPTURE_UPVAL + TIM2CH4_CAPTURE_DOWNVAL;
				// 若TIM2定时器溢出了，则DOWNVAL < UPVAL，则由65535 - UP + DOWN = 从上升沿到下降沿的计数 
			else uptimech4 = TIM2CH4_CAPTURE_DOWNVAL - TIM2CH4_CAPTURE_UPVAL; // 若未溢出，则上升沿到下降沿的计数= DOWN - UP
			TIM2CH4_CAPTURE_STA = 0; // 已经完成本次捕获，清除标志位
			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising);
		}					
		else // 如果之前没有检测到上升沿（这次是第一次检测到上升沿）
		{
			TIM2CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM2);
			TIM2CH4_CAPTURE_STA |= 0X40; // 第6位置位，代表已经检测到一次上升沿
			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling); // 设置CH4开始等待下降沿
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

