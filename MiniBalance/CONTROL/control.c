 #include "control.h"	
#include "filter.h"
extern float Target_Velocity;
extern float Turn_Target;
extern float Movement;
extern float Velocity_Kp,Velocity_Ki;
extern float Yaw;
#define L()  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)
#define LC() GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)
#define RC() GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)
#define R()  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
int L  = 0;
int LC = 0;
int RC = 0;
int R  = 0;  
int stop_3s_flag = 0;//停三秒标志位
int stop_3s_count = 0;//全部为白色检测计数
int stop_3s_finish = 0;//停车完成标志位
int slow_speed = 0;//减速标志位
int SPEED_LINE = 15000;
int SPEED_FULL =  7000;
int SPEED_TURN = 6000;
int SPEED_SLOW = -1000;
int count = 0;
int count_line = 0;
int check_flag = 5;//红外检测开启标志位

int flag_start = 0;//启动标志位

int tishi_count = 0;//0.5秒提示一次,6次为1秒
int zuihou_zhizou_finished = 0;
int zuihou_zhizou_count = 0;
int zuihou_into_lib_count = 0;
int last_tishi = 0;
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
 u8 Flag_Target;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
int Check_Count = 0;
int  times  =1;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{    
if(INT==0)//溢出中断
	{   
		   EXTI->PR=1<<12;                                                      //清除中断标志位     
       Flag_Target=!Flag_Target;
			 if(Flag_Target==0)
			 {                                             
	  	  Get_Angle(Way_Angle);                                               //===更新姿态		
				Voltage_Temp=Get_battery_volt();		                                //=====读取电池电压		
				Voltage_Count++;                                                    //=====平均值计数器
				Voltage_All+=Voltage_Temp;                                          //=====多次采样累积
				if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====求平均值		
       	Key();                                                               //===采集按键变化     
			//	Led_Flash(200);                                      								 //===LED闪烁;常规模式 1s改变一次指示灯的状态	
				return 0;
			 }	
								if(check_flag == 0)	 
								{
									L  = L();
									LC = LC();
									RC = RC();
									R  = R();
								
									Check_Count++;
									if(flag_start == 1)
										count_line++;
									if(count_line == 50)
									 {
											SPEED_LINE=50000;
										 Velocity_Kp=105;
										 Velocity_Ki=1.05;
									 }
										 
									 
									 if(count_line==800)
									 {
											Velocity_Kp=101;
										 Velocity_Ki=1;
									 }
								 if(Check_Count>times)
								 {
									 Check_Count=0;
									 
									 check();
								 }
							 }
							if(stop_3s_flag == 1)
							{
								TIM3_Int_Init(4999,7199);
								stop_3s_flag =0;
							}
							 if(stop_3s_finish ==1)
							 {
									Target_Velocity = 15000;
									Flag_Left = 0;
									Flag_Right = 0;
									TIM2_Int_Init(4999,7199);
									stop_3s_finish =0;//关闭标志位
							 }
							 
							 
							 if(zuihou_zhizou_finished ==1)
							 {
								 Check_Count++;
								 if(Check_Count>times)
								 {
									 Check_Count=0;
									 check_into_lib();
								 }
							 }
				

			 Get_Angle(Way_Angle);                                                 //===更新姿态	
 		   Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                     //===平衡控制	
		   Velocity_Pwm=velocity(Moto1,Moto2); 			 //===速度环PI控制	 速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点    
			 Turn_Pwm =turn(Moto1,Moto2);
	     Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                              //===计算左轮电机最终PWM
       Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                              //===计算右轮电机最终PWM
	     
			 Xianfu_Pwm();
			 if(Turn_Off(Angle_Balance,Voltage)==0)                                //===如果不存在异常
 			 Set_Pwm(Moto1,Moto2);                                                 //===赋值给PWM寄存器  
			 else
	     Set_Pwm(0,0);           
       Get_Zhongzhi	();		 
   	}       	
	 return 0;	  
} 
/**************************************************************************
函数功能：直立PD控制
入口参数：角度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle ,float Gyro)
{  
   float Bias;    //这里D为零
	 int balance;
	 Bias=Angle-Zhongzhi;          //===求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Balance_Kd*Gyro;      //===计算平衡控制的电机PWM 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int velocity_left,int velocity_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	 // static float Target_Velocity=100000;
	  static float Encoder_Integral;  
		Movement=Target_Velocity;
//		if(1==Flag_Qian)    	Movement=Target_Velocity;	           //===前进标志位置1 
//		else if(1==Flag_Hou)	Movement=-Target_Velocity;           //===后退标志位置1
//  	else  Movement=0;
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //避障标志位置1且非遥控转弯的时候，进入避障模式
//    Movement=-Target_Velocity;
   //=============速度PI控制器=======================//	
		Encoder_Least=Mean_Filter(velocity_left,velocity_right);          //速度滤波  对前后十次速度只和求取平均值  
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 
		Encoder_Integral +=Movement;                                      //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>320000)  	Encoder_Integral=320000;            //===积分限幅
		if(Encoder_Integral<-320000)	Encoder_Integral=-320000;              //===积分限幅	
		Velocity=Encoder*Velocity_Kp/95+Encoder_Integral*Velocity_Ki/95;     //===速度控制	
  	//if(Turn_Off(Angle_Balance,Voltage)==1)    Encoder_Integral=0;      //===电机关闭后清除积分
	  return Velocity;
}


/**************************************************************************
函数功能：赋值给PWM寄存器,并且判断转向
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
      if(moto1>0)			    Right_Direction=0;
			else 	              Right_Direction=1;
	   	if(moto2>0)			    Left_Direction=0;
			else 	              Left_Direction=1;
	    Final_Moto1=Linear_Conversion(moto1);  //线性化
    	Final_Moto2=Linear_Conversion(moto2);
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude_H=5000, Amplitude_L=-5000;       
    if(Moto1<Amplitude_L)  Moto1=Amplitude_L;	
		if(Moto1>Amplitude_H)  Moto1=Amplitude_H;	
	  if(Moto2<Amplitude_L)  Moto2=Amplitude_L;	
		if(Moto2>Amplitude_H)  Moto2=Amplitude_H;	
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(100); 

	if(tmp==1)
	{
			flag_start = 1;
		Flag_Stop=0;
		check_flag=0;
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	}//单击控制小车的启停
	if(tmp==2)Flag_Show=!Flag_Show;//双击控制小车的显示状态
	tmp2=Long_Press();                   
  if(tmp2==1) Bi_zhang=!Bi_zhang;		//长按控制小车是否进入超声波避障模式 
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	      u8 temp;
			if(angle<-40||angle>40||1==Flag_Stop||voltage<1100)//电池电压低于11.1V关闭电机
			{	                                                 //===倾角大于40度关闭电机
      temp=1;                                            //===Flag_Stop置1关闭电机
			Moto1=0;
      Moto2=0;		
			ST=0;	
      if(angle<-75||angle>75)                            //小车摔倒 停止标志位置1
			Flag_Stop=1;
      }
			else
			{	
      temp=0;
			ST=1;	
			}
      return temp;			
}
	
/**************************************************************************
函数功能：获取角度 
入口参数：获取角度的算法 1：DMP 
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	 //  float Accel_Y,Accel_Angle,Accel_Z,Gyro_X,Gyro_Z;
	   	Temperature=Read_Temperature();      //===读取MPU6050内置温度传感器数据，近似表示主板温度。
	    if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
			{	
					Read_DMP();                      //===读取加速度、角速度、倾角
					Angle_Balance=-Roll;             //===更新平衡倾角
				  Gyro_Balance=gyro[0];             //===更新平衡角速度
				  Acceleration_Z=accel[2];         //===更新Z轴加速度计
			}			

}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：速度滤波
入口参数：速度
返回  值：滤波后的速度   对前面10次速度求取平均值
**************************************************************************/
int Mean_Filter(int moto1,int moto2)
{
  u8 i;
  s32 Sum_Speed = 0; 
	s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =moto1+moto2;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
	return Filter_Speed;
}

/**************************************************************************
函数功能：对控制输出的PWM线性化,便于给系统寄存器赋值
入口参数：PWM
返回  值：线性化后的PWM
**************************************************************************/
u16  Linear_Conversion(int moto)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=36000000/(PRESCALER+1)/13000*5000/myabs(moto);//计算ARR的值  说明PID输出的motor的值乘以13000/5000为PWM输出频率
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	
/**************************************************************************
函数功能：自适应中值
入口参数：无
返回  值：无
**************************************************************************/
void  Get_Zhongzhi(void)
{
	    static int  count;
     if(Flag_Stop==0&&Flag_Zhongzhi==0)
		 {
		    if(myabs(Moto1)< 0&&myabs(Moto2)<100)  		count++;		//采样     21度
				else			                                	count=0;	
				if(count>300)  //连线3秒处于平衡位置，读取中值
				{	
				Zhongzhi=(int)Angle_Balance;	
				Flag_Zhongzhi=1;
				}
		 } 
}

int turn(int velocity_left,int velocity_right)//转向控制
{
		static float Turn_Amplitude=1200,Turn_Target,Turn_Convert=0.3;     
	  static float Turn_Count,Encoder_temp;
	  if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(velocity_left+velocity_right);
			Turn_Convert=43200/Encoder_temp;
			if(Turn_Convert<96)Turn_Convert=96;
			if(Turn_Convert>960)Turn_Convert=960;
		}	
	  else
		{
			Turn_Convert=6;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target-=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
	  return Turn_Target;
}

void check()
{
	if(L==0&&LC==0&&RC==0&&R==0)    //全部在黑线上     1      0为黑
	{
		count++;
		if(count>6)
		{
//			Target_Velocity = 0;
			SPEED_LINE = 550;
			SPEED_FULL = 550;
			SPEED_TURN = 550;
			slow_speed = 1;
			count=0;
		}
		
		Flag_Left = 0;
		Flag_Right = 0;
	}
	else if(L==0&&LC==0&&RC==0&&R==1)//微型右偏   右偏严重    2
	{
//		Target_Velocity = SPEED_LINE;

//		Velocity_Kp = 98;
//		Velocity_Ki = 0.85;
//		SPEED_LINE = 9000;
		
		if(slow_speed == 1)
			SPEED_LINE = 1000;
		Target_Velocity = SPEED_TURN;
		
		Flag_Left = 1;
		Flag_Right = 0;
	}
//	else if(L==0&&LC==0&&RC==1&&R==0)//不可能存在       3
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 1;
//		Flag_Right = 0;
//	}
	else if(L==0&&LC==0&&RC==1&&R==1)//微型右偏       4
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 1;
		Flag_Right = 0;
	}
//	else if(L==0&&LC==1&&RC==0&&R==0)//不可能存在     5
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 1;
//		Flag_Right = 0;
//	}
//	else if(L==0&&LC==1&&RC==0&&R==1)// 不可能存在      6
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 1;
//		Flag_Right = 0;
//	}
	
//	else if(L==0&&LC==1&&RC==1&&R==0)//不可能存在      7
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 0;
//		Flag_Right = 1;
//	}
		else if(L==0&&LC==1&&RC==1&&R==1)//右偏严重       8
	{
		Velocity_Kp = 98;
		Velocity_Ki = 0.85;
		SPEED_LINE = 9000;
		
		if(slow_speed == 1)
			SPEED_LINE = 1000;
		Target_Velocity = SPEED_TURN;
		Flag_Left = 1;
		Flag_Right = 0;
	}
	else if(L==1&&LC==0&&RC==0&&R==0)//微型左偏    左偏严重   9
	{
//		Target_Velocity = SPEED_LINE;
//		Velocity_Kp = 98;
//		Velocity_Ki = 0.85;
//		SPEED_LINE = 9000;
		
		if(slow_speed == 1)
			SPEED_LINE = 1000;
		Target_Velocity = SPEED_TURN;
		
		Flag_Left = 0;
		Flag_Right = 1;
	}
	
	else if(L==1&&LC==0&&RC==0&&R==1)//中间       10
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 0;
		Flag_Right = 0;
	}
//	else if(L==1&&LC==0&&RC==1&&R==0)//不可能存在       11
//	{
//		Target_Velocity = 0;
//		Flag_Left = 0;
//		Flag_Right = 0;
//	}
	
	else if(L==1&&LC==0&&RC==1&&R==1)//微型右偏      12
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 1;
		Flag_Right = 0;
	}
	else if(L==1&&LC==1&&RC==0&&R==0)//微型左偏       13
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 0;
		Flag_Right = 1;
	}
	else if(L==1&&LC==1&&RC==0&&R==1)//微型左偏       14
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 0;
		Flag_Right = 1;
	}
	else if(L==1&&LC==1&&RC==1&&R==0)//左偏严重       15
	{
		Velocity_Kp = 98;
		Velocity_Ki = 0.85;
		SPEED_LINE = 9000;
		
		if(slow_speed == 1)
			SPEED_LINE = 1000;
		Target_Velocity = SPEED_TURN;
		Flag_Left = 0; 
		Flag_Right = 1;
	}
	else if(L==1&&LC==1&&RC==1&&R==1)//全白色       16
	{
		
		Target_Velocity = 0;
		Flag_Left = 0;
		Flag_Right = 0;    //红外检测关闭之前左右转标志位复位,速度置零
		stop_3s_count++;
		if(stop_3s_count>35)//执行50次检测每次检测时间间隔为10ms    500ms
		{
			stop_3s_flag = 1;//说明到达停车区域
			check_flag = 1;//红外检测关闭
		}
	}
	
	else{
		Flag_Left=0;
		Flag_Right=0;
	}
}



void check_into_lib()
{
	if ((Yaw>0&&Yaw<=180)||(Yaw<-173&&Yaw>=-180))
	{
		Flag_Left=1;
		Flag_Right=0;
	}
	else if(Yaw<0&&Yaw>-168)
	{
		Flag_Left=0;
		Flag_Right=1;
	}
	else
	{
		Flag_Left=0;
		Flag_Right=0;
	}
}


