#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
#include "stdlib.h"	    	
#define OLED_SDA_IN()  {GPIOC->CRH&=0XF0FFFFFF;GPIOC->CRH|=(u32)8<<24;}
#define OLED_SDA_OUT() {GPIOC->CRH&=0XF0FFFFFF;GPIOC->CRH|=(u32)3<<24;}

//IO操作函数	 
#define OLED_IIC_SCL    PCout(13) //SCL
#define OLED_IIC_SDA    PCout(14) //SDA	 
#define OLED_READ_SDA   PCin(14)  //输入SDA 


#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	 


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_IIC_Init(void);                //初始化IIC的IO口				 
void OLED_IIC_Start(void);				//发送IIC开始信号
void OLED_IIC_Stop(void);	  			//发送IIC停止信号
void OLED_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 OLED_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 OLED_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void OLED_IIC_Ack(void);					//IIC发送ACK信号
void OLED_IIC_NAck(void);				//IIC不发送ACK信号

void OLED_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 OLED_IIC_Read_One_Byte(u8 daddr,u8 addr);	 


void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void fill_picture(unsigned char fill_Data);
void Picture(void);
void OLED_IIC_Start(void);
void OLED_IIC_Stop(void);
void OLED_Write_IIC_Command(unsigned char IIC_Command);
void OLED_Write_IIC_Data(unsigned char IIC_Data);
void OLED_Write_IIC_Byte(unsigned char IIC_Byte);

void OLED_fuhao_write(unsigned char x,unsigned char y,unsigned char asc);
void OLED_Num_write(unsigned char x,unsigned char y,unsigned char asc) ;
void OLED_Float(unsigned char Y,unsigned char X,double real,unsigned char N);
void OLED_Float2(unsigned char Y,unsigned char X,double real,unsigned char N1,unsigned char N2);
void OLED_Num2(unsigned char x,unsigned char y, int number);
void OLED_Num3(unsigned char x,unsigned char y,int number); 
void OLED_Num4(unsigned char x,unsigned char y, int number);
void OLED_Num5(unsigned char x,unsigned char y,unsigned int number);
#endif  

