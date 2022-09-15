/************************************************************************************
								901
*************************************************************************************/
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "mpu6050.h"
//#include "usmart.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "data_transfer.h"
#include "delay.h"
#include "timer.h"
#include "pwm.h"
#include "schedule.h"
#include "GPIO.h"
#include "DRV8874.h"
#include "adc.h"


int a=150,b=150,c=150;
extern int16_t HIGH,highset;//实际高度
extern int32_t highTag, highNow, control;//定义一个目标速度，采样速度，控制量
int time2 = 0;
int time3 = 0;
int time4 = 0;
int time5 = 0;
int time6 = 0;
int time7 = 0;

int agl_count = 0;
float agl_sum0 = 0;
//PID_AbsoluteType dir_pd;
//int encoderCount=0;
int encoderCount=0;
int te1 =0;
//int lTest, rTest;
int test1=150, test2=150, test3=0;
int pre_count = 0;
int main(void)
{
    delay_init();
    NVIC_Configuration();
//		dir_pd.kp = 1.5;
//		dir_pd.ctrOut =0;
//	  dir_pd.kd = 10;
	  whole_encoder =0;
		uart_init(115200);        //串口初始化为9600
	  //time2_init();
	  //Adc_Init();	
	  delay_ms(300);
		// encoderCount=Get_Adc_Average(ADC_Channel_0,10);
		// te1 = 100*encoderCount*(3.3/4096);
	  // uart_init(9600);
    // TIM2_Cap_Init(0xffff, 72 - 1);	
	  // time2_init();
	  // DRV8874_Init();
		// delay_ms(3000);
		// encoderCount=Get_Adc_Average(ADC_Channel_0,10);
		// te1 = 100*encoderCount*(3.3/4096);
    TIM4_PWM_Init(299, 719);
		TIM2_PWM_Init(299, 719);
		// TIM_SetCompare2(TIM4, 0); // CH3 LEFT
		// TIM_SetCompare3(TIM4, 0); // CH3 LEFT
    // TIM_SetCompare4(TIM4, 150); // CH4 RTGHT
		// TIM3_PWM_Init(299, 719);
		// TIM_SetCompare1(TIM3, 0); // CH3 LEFT
		// TIM_SetCompare2(TIM3, 0); // CH3 LEFT
		TIM_SetCompare2(TIM4, 150); // PB7
		TIM_SetCompare3(TIM4, 150); // PB8
		TIM_SetCompare4(TIM4, 200); // PB9  电调
		TIM_SetCompare1(TIM2, 150); // PB7
		TIM_SetCompare2(TIM2, 150); // PB7
		MPU_Init();                 //初始化MPU6050
	  // GPIO_init();
		PID_ControlInit();
	
//	  PID_Control.kp = 1;
//	  PID_Control.kd = 0.2;    
//    PID_Control.ki = 0;
		
		// PID_Control_1.kp = 100;
		// PID_Control_1.kd = 5;    
		// PID_Control_1.ki = 0;
		
		
	
    delay_ms(3000); // 保持平展3s
    TIM1_Int_Init(19,7199);	//10ms中断 //72000000

		while(mpu_dmp_init())//保证姿态姿态传感器开启
    {
        //printf("mpu6050 error!\n");
        delay_ms(200);
    }
    //printf("MPU6050 OK");
		
		
    while(1)
    {
			
			//a控制方向，b控制转速 0-1000，1000最快
			//a= ndef;
			//TIM_SetCompare1(TIM4, a);
			
				//encoderCount=Get_Adc_Average(ADC_Channel_0,10);
				
				//te1 = 100*encoderCount*(3.3/4096);
			  
        if(mpu_dmp_get_data(&roll,&pitch,&yaw)==0)		//耗时3ms
        {
					
					
            //temp=MPU_Get_Temperature(); //得到温度值	//耗时约1ms
					
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz);   //得到加速度传感器数据	//耗时约1ms
					
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //得到陀螺仪数据		//耗时约1m
					  //ANO_DT_Send_Status(roll,pitch,yaw,0,time1/25);
				}
				if(send_flag == 1)
				{
					
					//ANO_DT_Send_Status(roll,pitch,yaw,0,time1/25);
					te1 = 330*encoderCount/4096;
					
					pre_count = te1;
					//ANO_DT_Send_Status(roll,pitch,yaw,0,time1/25); //往遥控发数
					send_flag = 0;
				}
				ANO_DT_Data_Exchange();
				//TIM_SetCompare2(TIM4, test1);
				//TIM_SetCompare3(TIM4, test2);
				//TIM_SetCompare1(TIM2, test1);
				//TIM_SetCompare2(TIM2, test2);

    }

}





