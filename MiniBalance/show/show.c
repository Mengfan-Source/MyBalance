//#include "show.h"
//  /**************************************************************************
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//**************************************************************************/
//unsigned char i;          //��������
//unsigned char Send_Count; //������Ҫ���͵����ݸ���
//float Vol;
///**************************************************************************
//�������ܣ�OLED��ʾ
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void oled_show(void)
//{
//		//=============��һ����ʾС��ģʽ=======================//	
//		     if(Way_Angle==1)	OLED_ShowString(0,0,"DMP");
//		else if(Way_Angle==2)	OLED_ShowString(0,0,"Kalman");
//		else if(Way_Angle==3)	OLED_ShowString(0,0,"Hubu");
//                   
//	       if(Bi_zhang==1)	OLED_ShowString(60,0,"Bizhang");
//		     else             OLED_ShowString(60,0,"Putong");
//	  //=============�ڶ�����ʾ�¶Ⱥ;���===============//	
//													OLED_ShowNumber(0,10,Temperature/10,2,12);
//													OLED_ShowNumber(23,10,Temperature%10,1,12);
//													OLED_ShowString(13,10,".");
//													OLED_ShowString(35,10,"`C");
//													OLED_ShowNumber(70,10,(u16)Distance,5,12);
//			                    OLED_ShowString(105,10,"mm");
//		//=============��������ʾ������1=======================//	
//		                      OLED_ShowString(00,20,"Moto1");
//		if( Moto1<0)		OLED_ShowString(80,20,"-"),
//		                      OLED_ShowNumber(95,20,-Moto1,5,12);
//		else                 	OLED_ShowString(80,20,"+"),
//		                      OLED_ShowNumber(95,20,Moto1,5,12);
//  	//=============��������ʾ������2=======================//		
//		                      OLED_ShowString(00,30,"Moto2");
//		if(Moto2<0)		        OLED_ShowString(80,30,"-"),
//		                      OLED_ShowNumber(95,30,-Moto2,5,12);
//		else               		OLED_ShowString(80,30,"+"),
//		                      OLED_ShowNumber(95,30,Moto2,5,12);	
//		//=============��������ʾ��ѹ=======================//
//		                      OLED_ShowString(00,40,"Volta");
//		                      OLED_ShowString(58,40,".");
//		                      OLED_ShowString(80,40,"V");
//		                      OLED_ShowNumber(45,40,Voltage/100,2,12);
//		                      OLED_ShowNumber(68,40,Voltage%100,2,12);
//		 if(Voltage%100<10) 	OLED_ShowNumber(62,40,0,2,12);
//		//=============��������ʾ�Ƕ�=======================//
//		                      OLED_ShowString(0,50,"Angle");
//		if(Angle_Balance<0)		OLED_ShowNumber(45,50,Angle_Balance+360,3,12);
//		else					        OLED_ShowNumber(45,50,Angle_Balance,3,12);
//		if(Flag_Zhongzhi==0)	OLED_ShowString(110,50,"N");
//    if(Flag_Zhongzhi==1)  OLED_ShowString(110,50,"Y");
//		//=============ˢ��=======================//
//		OLED_Refresh_Gram();	
//	}
