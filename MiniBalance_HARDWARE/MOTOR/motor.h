#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

#define Left_Direction   PBout(15) 
#define Right_Direction  PBout(13) 
#define ST   PBout(14) 
void MiniBalance_PWM_Init(void);
void MiniBalance_Motor_Init(void);
#endif
