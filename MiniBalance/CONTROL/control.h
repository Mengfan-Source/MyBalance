#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define FILTERING_TIMES  10
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int EXTI15_10_IRQHandler(void);
int balance(float Angle ,float Gyro);
int velocity(int velocity_left,int velocity_right);
void Set_Pwm(int moto1,int moto2);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void speed_filter(void);
int Mean_Filter(int moto1,int moto2);
u16  Linear_Conversion(int moto);
void  Get_Zhongzhi(void);
int turn(int velocity_left,int velocity_right);//转向控制
void check(void);//红外寻迹
void check_into_lib(void);

#endif
