#include "sys.h"
#include "data_transfer.h"
#include "DRV8874.h"
#include "adc.h"

extern u16 L_PWM;// = 150; // ����
extern int R_PWM;// = 150; // ����
extern u16 T_PWM;// = 150; // ����

extern u32 uptimech1;
extern u32 uptimech2;
extern u32 uptimech3;
extern u32 uptimech4;

extern long int  time1;
extern int  send_flag;
extern float pitch,roll,yaw;       //ŷ����
extern short aacx,aacy,aacz;       //���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;    //������ԭʼ����
extern short temp;                 //�¶�

extern float agl0;
extern int agl_init;
extern float agl_initBuf[5];
extern int te1;

/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 float errILim;//����������
 
 float errNow;//��ǰ�����
 float ctrOut;//���������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;


/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 int32_t kp;     //����ϵ��
 int32_t ki;     //����ϵ��
 int32_t kd;     //΢��ϵ��
 
 int32_t errNow; //��ǰ�����
 int32_t dCtrOut;//�����������
 int32_t  ctrOut;//�������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 int32_t errOld1;
 int32_t errOld2;
 
}PID_IncrementType;

extern PID_AbsoluteType PID_Control;
extern PID_IncrementType PID_Control_1;
extern PID_AbsoluteType dir_pd;
extern int whole_encoder;
extern int32_t d_yaw_control,d_pitch_control,d_roll_control ;
extern int32_t dir_pwm_pitch, dir_pwm_yaw;
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM1_Int_Init(u16 arr,u16 psc);
void PID_ControlInit(void);
void TIM1_UP_IRQHandler(void);
void PID_AbsoluteMode(PID_AbsoluteType* PID);
void PID_IncrementMode(PID_IncrementType* PID);
void PID();