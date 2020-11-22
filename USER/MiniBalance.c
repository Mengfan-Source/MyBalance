#include "stm32f10x.h"
#include "sys.h"

  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1 ;                             //��ȡ�Ƕȵ��㷨�� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=1; //����ң����صı���
u8 Flag_Stop=0,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Moto1,Moto2,Final_Moto1,Final_Moto2;    //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Roll,Angle_Balance,Gyro_Balance;      //
float Show_Data_Mb=0,Show_Data_Mb2=0;       //ȫ����ʾ������������ʾ��Ҫ�鿴������
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //��ʱ�͵��εȱ���
u32 Distance;                               //������
float Acceleration_Z;                       //Z����ٶȼ�  
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //��ģң�ؽ��ձ���
float Balance_Kp=350,Balance_Kd=0,Velocity_Kp=105,Velocity_Ki=1.05;//PID����
int Zhongzhi,Flag_Zhongzhi=0;
extern int L,LC,RC,R;

	float Target_Velocity =0 ;
	extern int times,SPEED_TURN,SPEED_FULL;
	extern float Yaw;
	extern int count,count_line;
int main(void)
  { 
		delay_init();	    	            //=====��ʱ������ʼ��	
		uart_init(128000);	            //=====���ڳ�ʼ��Ϊ12800������
		//uart3_init(9600);	            //=====���ڳ�ʼ��Ϊ9600������     ʹ�õ�ʱ���ж����ȼ���ʱ��3��ͻ
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
		LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
		Buz_Init();
	  KEY_Init();                     //=====������ʼ��
		KEY1_Init(); 
		MY_NVIC_PriorityGroupConfig(2); //=====�жϷ���
		delay_ms(100);                  //=====��ʱ�ȴ�
		Adc_Init();                     //=====adc��ʼ��
		delay_ms(100);                  //=====��ʱ�ȴ�
    IIC_Init();                     //=====IIC��ʼ��
    MPU6050_initialize();           //=====MPU6050��ʼ��	
    DMP_Init();                     //=====��ʼ��DMP 	
		OLED_IIC_Init();
		OLED_Init();
		OLED_Clear();    
		MiniBalance_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��
		MiniBalance_Motor_Init();       //=====�������IO���Ƴ�ʼ��
    MiniBalance_PWM_Init();         //=====��ʼ��PWM ���
    while(1)
	   {
//			 OLED_ShowString(0,0,"V:",12);
//			 OLED_Float(0,50,Target_Velocity,3);
//			 
//			 OLED_ShowString(0,1,"KP:",12);
//			 OLED_Float(1,50,Velocity_Kp,3);
//			 
//			 OLED_ShowString(0,2,"KI:",12);
//			 OLED_Float(2,50,Velocity_Ki,3);
//			 
//			 OLED_ShowString(0,3,"times:",12);
//			 OLED_Float(3,50,times,3);
			 
			 OLED_ShowString(0,0,"L:",12);
			 OLED_Float(0,50,count,3);
			 
			 OLED_ShowString(0,1,"LC:",12);
			 OLED_Float(1,50,Velocity_Kp,3);
			 
			 OLED_ShowString(0,2,"RC:",12);
			 OLED_Float(2,50,count_line,3);
			 
			 OLED_ShowString(0,3,"R:",12);
			 OLED_Float(3,50,R,3);
			 
			 
			 OLED_ShowString(0,4,"flagL:",12);
			 OLED_Float(4,50,Flag_Left,3);
			 
			 OLED_ShowString(0,5,"flagR:",12);
			 OLED_Float(5,50,Flag_Right,3);
			 
//			 OLED_ShowString(0,4,"FULL:",12);
//			 OLED_Float(4,50,SPEED_FULL,3);
//			 
//			 OLED_ShowString(0,5,"TURN:",12);
//			 OLED_Float(5,50,SPEED_TURN,3);
//			 OLED_ShowString(0,0,"Yaw:",12);
//			 OLED_Float(0,50,Yaw,3);
	   } 
}

