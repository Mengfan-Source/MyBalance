#ifndef __LED_H
#define __LED_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
//LED 端口定义
#define LED PAout(4)  
#define BUZ PCout(15)
void LED_Init(void);  //初始化
void Led_Flash(u16 time);
void Buz_Init(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM2_Int_Init(u16 arr,u16 psc);//3秒时间计时
#endif
