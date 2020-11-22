#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

#define Left_Direction   PBout(15) 
#define Right_Direction  PBout(13) 
#define ST   PBout(14) 
void MiniBalance_PWM_Init(void);
void MiniBalance_Motor_Init(void);
#endif
