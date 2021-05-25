#include "sys.h"
#include "usart.h"	 
#include "stdarg.h"	 
#include "stdio.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
////加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 

//}; 

//FILE   __stdout;       
////定义_sys_exit()以避免使用半主机模式    
//_sys_exit(int x) 
//{ 
//	x = x; 
//	
//} 
////重定义fputc函数 
//int std :: fputc(int ch, FILE *f)
//{      
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
//    USART_SendData(USART1,(uint8_t)ch);   
//	return ch;
//}
//#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
 /*
*函数名 :itoa
*描述   :将整形数据转化为字符串
*输入   :-radix =10 表示10进制，其他结果为0
					-value 要转换的整型数
					-buf 转换后的字符串
					-radix =10
*输出		:无
*返回		:无
*调用		:被 USARTx_printf()调用
*/
static char *itoa(int value, char *string, int radix)
{
	int  i,d;
	int  flag=0;
	char  *ptr=string;
//仅工作在10进制下
	if(radix!=10)
	{
		*ptr=0;
		return string;
	}
	if(!value)   //是0  显示字符0
	{
		*ptr++=0x30;
		*ptr	=0;
		return string;	
	}
//添加“-”
	if(value<0)
	{
			*ptr++='-';
			value *=-1; //value 取反
	}
	for(i=10000;i>0;i/=10)
	{
		d=value/i;
		if(d|| flag)
		{
			*ptr++=(char)(d+0x30);
			value-=(d*i);
			flag=1;		
		}	
	}
	//空值
	*ptr=0;
	return string;
}
/*
*函数名 :USARTx_printf
*描述   :格式化输出，类似于printf,但未用到c库
*输入   :-USARTx 串口通道
					-Data 发送内容指针
					-...其他参数
*输出		:无
*返回		:无
*调用		:外部调用
					典型应用：USARTx_printf( USART1,"this is a demo\r\n");
										USARTx_printf( USART1,"%d\r\n", i);
*/
void USARTx_printf(USART_TypeDef* USARTx,const char *Data,...)
{
	const char *s;
	int d;
	char buf[16];
	va_list ap;
	va_start(ap, Data);
	while(*Data !=0)
	{
		if(*Data ==0x5c) //'/'
		{
			switch( *++Data )
			{
				case 'r':
							USART_SendData(USARTx, 0x0d);
							Data++;
							break;
				case 'n':
							USART_SendData(USARTx, 0x0a);
							Data++;
							break;
				default:
							Data++;
							break;
			}
		}
		else if(*Data =='%')
		{
			switch( *++Data )
			{
				case 's':
							s=va_arg(ap, const char*);
							for(;*s ; s++)
							{
								USART_SendData(USARTx, *s);
								while( USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
							}
							Data++;
							break;
				case 'd':
							d=va_arg(ap, int);
							itoa(d,buf,10);
							for(s=buf;*s ; s++)
							{
								USART_SendData(USARTx, *s);
								while( USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
							}
							Data++;
							break;
				default:
							Data++;
							break;
			}
		
		} //end of else if
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}
 
 
 
 
 
 
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
 	USART_DeInit(USART1);  //复位串口1
	 //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10


   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口
#if EN_USART1_RX		  //如果使能了接收  
   //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
#endif
    USART_Cmd(USART1, ENABLE);                    //使能串口 

}

#ifdef __cplusplus
extern "C" {
#endif

void USART1_IRQHandler(void)                	//串口1中断服务程序
//void USART1_IRQHandler(void) 
	{
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif
} 
	
#ifdef __cplusplus
}
#endif

#endif	



