#include "stm32f10x.h"
#include "sys.h"

  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1 ;                             //获取角度的算法， 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=1; //蓝牙遥控相关的变量
u8 Flag_Stop=0,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Moto1,Moto2,Final_Moto1,Final_Moto2;    //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Roll,Angle_Balance,Gyro_Balance;      //
float Show_Data_Mb=0,Show_Data_Mb2=0;       //全局显示变量，用于显示需要查看的数据
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //延时和调参等变量
u32 Distance;                               //测距变量
float Acceleration_Z;                       //Z轴加速度计  
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //航模遥控接收变量
float Balance_Kp=350,Balance_Kd=0,Velocity_Kp=105,Velocity_Ki=1.05;//PID参数
int Zhongzhi,Flag_Zhongzhi=0;
extern int L,LC,RC,R;

	float Target_Velocity =0 ;
	extern int times,SPEED_TURN,SPEED_FULL;
	extern float Yaw;
	extern int count,count_line;
int main(void)
  { 
		delay_init();	    	            //=====延时函数初始化	
		uart_init(128000);	            //=====串口初始化为12800波特率
		//uart3_init(9600);	            //=====串口初始化为9600波特率     使用的时候中断优先级定时器3冲突
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
		LED_Init();                     //=====初始化与 LED 连接的硬件接口
		Buz_Init();
	  KEY_Init();                     //=====按键初始化
		KEY1_Init(); 
		MY_NVIC_PriorityGroupConfig(2); //=====中断分组
		delay_ms(100);                  //=====延时等待
		Adc_Init();                     //=====adc初始化
		delay_ms(100);                  //=====延时等待
    IIC_Init();                     //=====IIC初始化
    MPU6050_initialize();           //=====MPU6050初始化	
    DMP_Init();                     //=====初始化DMP 	
		OLED_IIC_Init();
		OLED_Init();
		OLED_Clear();    
		MiniBalance_EXTI_Init();        //=====MPU6050 5ms定时中断初始化
		MiniBalance_Motor_Init();       //=====电机方向IO控制初始化
    MiniBalance_PWM_Init();         //=====初始化PWM 输出
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

