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
int stop_3s_flag = 0;//ͣ�����־λ
int stop_3s_count = 0;//ȫ��Ϊ��ɫ������
int stop_3s_finish = 0;//ͣ����ɱ�־λ
int slow_speed = 0;//���ٱ�־λ
int SPEED_LINE = 15000;
int SPEED_FULL =  7000;
int SPEED_TURN = 6000;
int SPEED_SLOW = -1000;
int count = 0;
int count_line = 0;
int check_flag = 5;//�����⿪����־λ

int flag_start = 0;//������־λ

int tishi_count = 0;//0.5����ʾһ��,6��Ϊ1��
int zuihou_zhizou_finished = 0;
int zuihou_zhizou_count = 0;
int zuihou_into_lib_count = 0;
int last_tishi = 0;
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
 u8 Flag_Target;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
int Check_Count = 0;
int  times  =1;
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{    
if(INT==0)//����ж�
	{   
		   EXTI->PR=1<<12;                                                      //����жϱ�־λ     
       Flag_Target=!Flag_Target;
			 if(Flag_Target==0)
			 {                                             
	  	  Get_Angle(Way_Angle);                                               //===������̬		
				Voltage_Temp=Get_battery_volt();		                                //=====��ȡ��ص�ѹ		
				Voltage_Count++;                                                    //=====ƽ��ֵ������
				Voltage_All+=Voltage_Temp;                                          //=====��β����ۻ�
				if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====��ƽ��ֵ		
       	Key();                                                               //===�ɼ������仯     
			//	Led_Flash(200);                                      								 //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
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
									stop_3s_finish =0;//�رձ�־λ
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
				

			 Get_Angle(Way_Angle);                                                 //===������̬	
 		   Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                     //===ƽ�����	
		   Velocity_Pwm=velocity(Moto1,Moto2); 			 //===�ٶȻ�PI����	 �ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��    
			 Turn_Pwm =turn(Moto1,Moto2);
	     Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                              //===�������ֵ������PWM
       Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                              //===�������ֵ������PWM
	     
			 Xianfu_Pwm();
			 if(Turn_Off(Angle_Balance,Voltage)==0)                                //===����������쳣
 			 Set_Pwm(Moto1,Moto2);                                                 //===��ֵ��PWM�Ĵ���  
			 else
	     Set_Pwm(0,0);           
       Get_Zhongzhi	();		 
   	}       	
	 return 0;	  
} 
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance(float Angle ,float Gyro)
{  
   float Bias;    //����DΪ��
	 int balance;
	 Bias=Angle-Zhongzhi;          //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Balance_Kd*Gyro;      //===����ƽ����Ƶĵ��PWM 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity
��ڲ����������ٶȡ������ٶ�
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity(int velocity_left,int velocity_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	 // static float Target_Velocity=100000;
	  static float Encoder_Integral;  
		Movement=Target_Velocity;
//		if(1==Flag_Qian)    	Movement=Target_Velocity;	           //===ǰ����־λ��1 
//		else if(1==Flag_Hou)	Movement=-Target_Velocity;           //===���˱�־λ��1
//  	else  Movement=0;
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
//    Movement=-Target_Velocity;
   //=============�ٶ�PI������=======================//	
		Encoder_Least=Mean_Filter(velocity_left,velocity_right);          //�ٶ��˲�  ��ǰ��ʮ���ٶ�ֻ����ȡƽ��ֵ  
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� 
		Encoder_Integral +=Movement;                                      //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>320000)  	Encoder_Integral=320000;            //===�����޷�
		if(Encoder_Integral<-320000)	Encoder_Integral=-320000;              //===�����޷�	
		Velocity=Encoder*Velocity_Kp/95+Encoder_Integral*Velocity_Ki/95;     //===�ٶȿ���	
  	//if(Turn_Off(Angle_Balance,Voltage)==1)    Encoder_Integral=0;      //===����رպ��������
	  return Velocity;
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���,�����ж�ת��
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
      if(moto1>0)			    Right_Direction=0;
			else 	              Right_Direction=1;
	   	if(moto2>0)			    Left_Direction=0;
			else 	              Left_Direction=1;
	    Final_Moto1=Linear_Conversion(moto1);  //���Ի�
    	Final_Moto2=Linear_Conversion(moto2);
}
/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
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
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
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
	}//��������С������ͣ
	if(tmp==2)Flag_Show=!Flag_Show;//˫������С������ʾ״̬
	tmp2=Long_Press();                   
  if(tmp2==1) Bi_zhang=!Bi_zhang;		//��������С���Ƿ���볬��������ģʽ 
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	      u8 temp;
			if(angle<-40||angle>40||1==Flag_Stop||voltage<1100)//��ص�ѹ����11.1V�رյ��
			{	                                                 //===��Ǵ���40�ȹرյ��
      temp=1;                                            //===Flag_Stop��1�رյ��
			Moto1=0;
      Moto2=0;		
			ST=0;	
      if(angle<-75||angle>75)                            //С��ˤ�� ֹͣ��־λ��1
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
�������ܣ���ȡ�Ƕ� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP 
����  ֵ����
**************************************************************************/
void Get_Angle(u8 way)
{ 
	 //  float Accel_Y,Accel_Angle,Accel_Z,Gyro_X,Gyro_Z;
	   	Temperature=Read_Temperature();      //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	    if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
			{	
					Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
					Angle_Balance=-Roll;             //===����ƽ�����
				  Gyro_Balance=gyro[0];             //===����ƽ����ٶ�
				  Acceleration_Z=accel[2];         //===����Z����ٶȼ�
			}			

}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ��ٶ��˲�
��ڲ������ٶ�
����  ֵ���˲�����ٶ�   ��ǰ��10���ٶ���ȡƽ��ֵ
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
�������ܣ��Կ��������PWM���Ի�,���ڸ�ϵͳ�Ĵ�����ֵ
��ڲ�����PWM
����  ֵ�����Ի����PWM
**************************************************************************/
u16  Linear_Conversion(int moto)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=36000000/(PRESCALER+1)/13000*5000/myabs(moto);//����ARR��ֵ  ˵��PID�����motor��ֵ����13000/5000ΪPWM���Ƶ��
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	
/**************************************************************************
�������ܣ�����Ӧ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Get_Zhongzhi(void)
{
	    static int  count;
     if(Flag_Stop==0&&Flag_Zhongzhi==0)
		 {
		    if(myabs(Moto1)< 0&&myabs(Moto2)<100)  		count++;		//����     21��
				else			                                	count=0;	
				if(count>300)  //����3�봦��ƽ��λ�ã���ȡ��ֵ
				{	
				Zhongzhi=(int)Angle_Balance;	
				Flag_Zhongzhi=1;
				}
		 } 
}

int turn(int velocity_left,int velocity_right)//ת�����
{
		static float Turn_Amplitude=1200,Turn_Target,Turn_Convert=0.3;     
	  static float Turn_Count,Encoder_temp;
	  if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
	  return Turn_Target;
}

void check()
{
	if(L==0&&LC==0&&RC==0&&R==0)    //ȫ���ں�����     1      0Ϊ��
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
	else if(L==0&&LC==0&&RC==0&&R==1)//΢����ƫ   ��ƫ����    2
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
//	else if(L==0&&LC==0&&RC==1&&R==0)//�����ܴ���       3
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 1;
//		Flag_Right = 0;
//	}
	else if(L==0&&LC==0&&RC==1&&R==1)//΢����ƫ       4
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 1;
		Flag_Right = 0;
	}
//	else if(L==0&&LC==1&&RC==0&&R==0)//�����ܴ���     5
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 1;
//		Flag_Right = 0;
//	}
//	else if(L==0&&LC==1&&RC==0&&R==1)// �����ܴ���      6
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 1;
//		Flag_Right = 0;
//	}
	
//	else if(L==0&&LC==1&&RC==1&&R==0)//�����ܴ���      7
//	{
//		Target_Velocity = SPEED_TURN;
//		Flag_Left = 0;
//		Flag_Right = 1;
//	}
		else if(L==0&&LC==1&&RC==1&&R==1)//��ƫ����       8
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
	else if(L==1&&LC==0&&RC==0&&R==0)//΢����ƫ    ��ƫ����   9
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
	
	else if(L==1&&LC==0&&RC==0&&R==1)//�м�       10
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 0;
		Flag_Right = 0;
	}
//	else if(L==1&&LC==0&&RC==1&&R==0)//�����ܴ���       11
//	{
//		Target_Velocity = 0;
//		Flag_Left = 0;
//		Flag_Right = 0;
//	}
	
	else if(L==1&&LC==0&&RC==1&&R==1)//΢����ƫ      12
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 1;
		Flag_Right = 0;
	}
	else if(L==1&&LC==1&&RC==0&&R==0)//΢����ƫ       13
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 0;
		Flag_Right = 1;
	}
	else if(L==1&&LC==1&&RC==0&&R==1)//΢����ƫ       14
	{
		Target_Velocity = SPEED_LINE;
		Flag_Left = 0;
		Flag_Right = 1;
	}
	else if(L==1&&LC==1&&RC==1&&R==0)//��ƫ����       15
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
	else if(L==1&&LC==1&&RC==1&&R==1)//ȫ��ɫ       16
	{
		
		Target_Velocity = 0;
		Flag_Left = 0;
		Flag_Right = 0;    //������ر�֮ǰ����ת��־λ��λ,�ٶ�����
		stop_3s_count++;
		if(stop_3s_count>35)//ִ��50�μ��ÿ�μ��ʱ����Ϊ10ms    500ms
		{
			stop_3s_flag = 1;//˵������ͣ������
			check_flag = 1;//������ر�
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


