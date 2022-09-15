#include "schedule.h"
#include <math.h>


float pitch,roll,yaw;       //ŷ����--MPU6050����
short aacx,aacy,aacz;       //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;    //�����ǽ��ٶ�ԭʼ����
short temp;                 //�¶�
int whole_encoder=0;
//ע�����ֿ�������߲�һ�£�������Ҫ����
//���Ǿɰ�
//#define PERIOD uptimech4
//#define OFFSET uptimech3
//#define TURNLR uptimech2
//�����°�
#define PERIOD uptimech2
#define TAILUD uptimech4
#define OFFSET uptimech3
#define TURNLR uptimech1
#define min(a,b) ((a<b)?a:b)
#define max(a,b) ((a>b)?a:b)


//void TIM2_Int_Init(u16 arr, u16 psc)
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��

//    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
//    TIM_TimeBaseStructure.TIM_Prescaler = psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

//    TIM_ITConfig(TIM2, TIM_IT_Update,	ENABLE); // ����TIM3�����ж�

//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3�ж�
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

//		TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
//}
void TIM1_Int_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��

  TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
  TIM_TimeBaseStructure.TIM_Prescaler =(psc-1);//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//??????
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//??????

  TIM_ITConfig(      //?????????TIM??
    TIM1,            //TIM1
    TIM_IT_Update  | //TIM ?????
    TIM_IT_Trigger,  //TIM ????? 
    ENABLE  	     //??
    );
	
  //?????
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//?????0?
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  	   //????0?
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  TIM_Cmd(TIM1, ENABLE);  //ʹ��TIMx����
}



extern int16_t Speed_set; // ���Ƶ��ҡ����������Ӧ��ֵ
extern int16_t D_yaw_control,D_pitch_control;  // �����������ҡ����������Ӧ��ֵ0~100
//int32_t d_yaw_control,d_pitch_control;        // ��D_yaw_control��0~100ת���������ĽǶȷ�Χ
int dir_pwm_yaw=150,dir_pwm_pitch = 150;    // �����������P��ֵ��150��Ӧ��ֵ



int32_t speed_set=0, speed_control=200;//���������
void User_PidSpeedControl(void)  //���Ƶ�������Ʒ��и߶�
{
    speed_set = Speed_set*1.10;
		speed_control=200 - speed_set;
		if(speed_control>200)
			speed_control=200;
		if(speed_control<90)
			speed_control=90;
		TIM_SetCompare4(TIM4, speed_control);
}



u16 yaw_Mid = 150;         //����yaw�������ֵ
u16 pitch_Mid = 150;       //����pitch�������ֵ
//int yaw_up_limit = 45;     //���������Ƕȵķ�Χ
//int yaw_down_limit = -45;  //���������Ƕȵķ�Χ
//int pitch_up_limit = 45;   //���������Ƕȵķ�Χ
//int pitch_down_limit = -45;//���������Ƕȵķ�Χ
PID_AbsoluteType yaw_PID;  				//�ǶȻ�PID������
PID_AbsoluteType pitch_PID;
PID_AbsoluteType yaw_speed_PID;		//���ٶȻ�PID������
PID_AbsoluteType pitch_speed_PID;
PID_AbsoluteType yaw_acc_PID;			//�Ǽ��ٶȻ�PID������
PID_AbsoluteType pitch_acc_PID;
int yaw_speed_limit = 20000;		 //���ٶ��޷�-21846~21846
int pitch_speed_limit = 20000;
int yaw_acc_limit = 20000;		 //�Ǽ��ٶ��޷�-21846~21846
int pitch_acc_limit = 20000;
int yaw_out_limit = 20;		 //�������޷���ֱ�������ڶ��P��
int pitch_out_limit = 5;
int target_yaw = 90; 			 //���������Ƕ�
int target_pitch = 0;
void Att_Control() //�����ȵ���̬����
{
	//yaw��̬�ǵ���������PID
	/************************************������ʱע��**************************************************/
	target_yaw = (D_yaw_control-50)*90/100;   //����ң������������D_yaw_control���������ĽǶȣ��޷�Ϊ-45~45����ֱ��ʱ��pitch��90����
	yaw_PID.errNow = target_yaw - roll;  //������yaw��ӦIMU��roll,
	/**************************************************************************************************/
	PID_AbsoluteMode(&yaw_PID);
	yaw_PID.ctrOut = yaw_PID.ctrOut>yaw_speed_limit? yaw_speed_limit:yaw_PID.ctrOut;
	yaw_PID.ctrOut = yaw_PID.ctrOut<-yaw_speed_limit? -yaw_speed_limit:yaw_PID.ctrOut;
		
	
	
	//pitch��̬�ǵ���������PID
	/************************************������ʱע��**************************************************/
	target_pitch	= (D_pitch_control-50)*90/100+100;
	pitch_PID.errNow = target_pitch - pitch;//������pitch��ӦIMU��pitch,��ֱ��ʱ��roll��0����
	/**************************************************************************************************/
	if(pitch_PID.errNow<5 && pitch_PID.errNow>-5)
		pitch_PID.kp = 30;
	else{
		pitch_PID.kp =80;
	}
	PID_AbsoluteMode(&pitch_PID);
	pitch_PID.ctrOut = pitch_PID.ctrOut>pitch_speed_limit? pitch_speed_limit:pitch_PID.ctrOut;
	pitch_PID.ctrOut = pitch_PID.ctrOut<-pitch_speed_limit? -pitch_speed_limit:pitch_PID.ctrOut;
	
}
void Att_Control_2() //�����ȵ���̬����
{
	yaw_speed_PID.errNow = yaw_PID.ctrOut - gyroz; //gyrox,y,z������������
	PID_AbsoluteMode(&yaw_speed_PID);
	yaw_speed_PID.ctrOut = yaw_speed_PID.ctrOut>yaw_out_limit? yaw_out_limit:yaw_speed_PID.ctrOut;
	yaw_speed_PID.ctrOut = yaw_speed_PID.ctrOut<-yaw_out_limit? -yaw_out_limit:yaw_speed_PID.ctrOut;
	dir_pwm_yaw = yaw_Mid + yaw_speed_PID.ctrOut;

	
	pitch_speed_PID.errNow = pitch_PID.ctrOut - gyrox; //gyrox,y,z������������
	PID_AbsoluteMode(&pitch_speed_PID);
	pitch_speed_PID.ctrOut = pitch_speed_PID.ctrOut>pitch_out_limit? pitch_out_limit:pitch_speed_PID.ctrOut;
	pitch_speed_PID.ctrOut = pitch_speed_PID.ctrOut<-pitch_out_limit? -pitch_out_limit:pitch_speed_PID.ctrOut;
	dir_pwm_pitch = pitch_Mid - pitch_speed_PID.ctrOut;
	if( dir_pwm_pitch>146 && dir_pwm_pitch<154)
		dir_pwm_pitch = 150;
	

	TIM_SetCompare2(TIM4, dir_pwm_yaw);  //Сβ��
	TIM_SetCompare3(TIM4, dir_pwm_pitch-D_pitch_control*100/50*0.5); //��β��  D_pitch_control*100/50*1.7
	
}



long int time1=0;
int send_flag = 0;

void TIM1_UP_IRQHandler(void) //2ms�ж�
{ 	    	  	     
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//?????TIM??????:TIM ??? 
	{
		User_PidSpeedControl(); //����
		Att_Control();    			//��̬����    
		time1++;
		if(time1>60000000)
			time1 = 0;
		if(time1%5 == 0) //10ms��һ��
			//send_flag = 1;
			Att_Control_2();
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}	 	
} 




//PID������ʼ��
void PID_ControlInit(void)
{
    //yaw
		yaw_PID.ctrOut = 0;
    yaw_PID.errNow = 0;
    yaw_PID.errOld = 0;
    yaw_PID.kd = 50;
    yaw_PID.kp = 100;
    yaw_PID.ki = 0;
	
		yaw_speed_PID.ctrOut = 0;
    yaw_speed_PID.errNow = 0;
    yaw_speed_PID.errOld = 0;
    yaw_speed_PID.kd = 0.02;
    yaw_speed_PID.kp = 0.05;
    yaw_speed_PID.ki = 0;
	
		yaw_acc_PID.ctrOut = 0;
    yaw_acc_PID.errNow = 0;
    yaw_acc_PID.errOld = 0;
    yaw_acc_PID.kd = 0.3;
    yaw_acc_PID.kp = 1;
    yaw_acc_PID.ki = 0;
		
		//pitch
		pitch_PID.ctrOut = 0;
    pitch_PID.errNow = 0;
    pitch_PID.errOld = 0;
    pitch_PID.kd = 30;
    pitch_PID.kp = 80;
    pitch_PID.ki = 0;
		
		pitch_speed_PID.ctrOut = 0;
    pitch_speed_PID.errNow = 0;
    pitch_speed_PID.errOld = 0;
    pitch_speed_PID.kd = 0.005;
    pitch_speed_PID.kp = 0.01;
    pitch_speed_PID.ki = 0;
		
		pitch_acc_PID.ctrOut = 0;
    pitch_acc_PID.errNow = 0;
    pitch_acc_PID.errOld = 0;
    pitch_acc_PID.kd = 0.3;
    pitch_acc_PID.kp = 1;
    pitch_acc_PID.ki = 0;
}



//λ��ʽPID�㷨
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
     if(PID->kp      < 0)    PID->kp      = -PID->kp;
     if(PID->ki      < 0)    PID->ki      = -PID->ki;
     if(PID->kd      < 0)    PID->kd      = -PID->kd;
     if(PID->errILim < 0)    PID->errILim = -PID->errILim;

     PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

     PID->errI += PID->errNow; //�����֣�����ki����

     PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

     PID->errOld = PID->errNow;	//�������ڵ����
     
     PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

}


/*******************************************************************************************************/

#define errDeath 10
//����ʽPID�㷨
void PID_IncrementMode(PID_IncrementType* PID)
{
    float dErrP, dErrI, dErrD;

    if(PID->kp < 0)    PID->kp = -PID->kp;
    if(PID->ki < 0)	PID->ki = -PID->ki;
    if(PID->kd < 0)    PID->kd = -PID->kd;

    dErrP = PID->errNow - PID->errOld1;

    dErrI = PID->errNow;

    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

    PID->errOld2 = PID->errOld1; //�������΢��
    PID->errOld1 = PID->errNow;  //һ�����΢��

    /*����ʽPID����*/
    //PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
    if((PID->errNow > 0 && PID->errNow < errDeath) || (PID->errNow > -errDeath && PID->errNow < 0))
    {
     ;//do nothing
    }
    else
    {
     if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   
         PID->ctrOut = 0;

     else 
         PID->ctrOut += PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
    }

//    if(PID->ctrOut > 90000)
//     PID->ctrOut = 90000;
//    else if(PID->ctrOut <= -90000)
//     PID->ctrOut = -90000;

    PID->ctrOut = PID->ctrOut/100;
}



//void TIM3_IRQHandler(void)   //TIM3�ж�
//{

//    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
//    {
//				time1++;
//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ

//    }
//}
