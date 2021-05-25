#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h"

#include "string.h"
#include "math.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
extern void USARTx_printf(USART_TypeDef* USARTx, const char *Data,...);

/*
#define BAUDRATE_NUM 7

uint32_t baudrate[BAUDRATE_NUM] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};

		USARTProc usartproc;
		usartproc.ReceiveNumber();
		if(usartproc.GetState()){
			USARTx_printf(usartproc.GetUsart(), "data=%d, str=%s \r\n", usartproc.GetData(), usartproc.GetReceiveStr());
		}
*/

class USARTProc{
public:
	USARTProc(){
		m_usart=USART1;
		m_bstate=false;
		memset(receive_data, '\0', RECIVE_DATA_MAX_LEN);
	}
	USARTProc(USART_TypeDef* usart){
		m_usart=usart;
		m_bstate=false;
		memset(receive_data, '\0', RECIVE_DATA_MAX_LEN);
	}
	
	int GetData() const{
		return m_data;
	}
	bool GetState() const{
		return m_bstate;
	}
	const char* GetReceiveStr(){
		return receive_data;
	}
	USART_TypeDef* GetUsart(){
		return m_usart;
	}
	
	// 通过串口获得数字
	void ReceiveNumber(){
    while(1){
			m_bstate=false;
			if(USART_RX_STA & 0x8000){
				int r, data; 						// 用于数据转换
				u16 len;
				bool bsign=true; 				// 获取数据的符号 true --> +, false --> -
				len = USART_RX_STA & 0x3fff;
				r = len;
        m_data = 0;
				memset(receive_data, '\0', RECIVE_DATA_MAX_LEN);
				
        for(int t = 0; t < len; t++){
					// 如果缺少以下两行代码会出现奇怪的BUG 连续输出两次数据
					USART_SendData(m_usart, USART_RX_BUF[t]);
					while(USART_GetFlagStatus(m_usart, USART_FLAG_TC) != SET){}
						receive_data[t] = USART_RX_BUF[t];
						if(receive_data[t]=='-'){
							bsign = false;
							data = 0;
						//}else if(isdigit(receive_data[t])){
						}else{
							data = (int)receive_data[t] - 48;
						}
						r = r - 1;
						m_data = m_data + data * (pow(10.0, r));
				}
				
				if(!bsign)
					m_data = -m_data;
				
				USART_RX_STA = 0;
				m_bstate=true;
				
				#ifdef DEBUG_INFO
					USARTx_printf(m_usart, "\r\n");
					USARTx_printf(m_usart, "data=%d, bsign=%d, bstate=%d \r\n", m_data, bsign, m_bstate);
				#endif
				
				break;
			}
		}
	}
private:
	enum {RECIVE_DATA_MAX_LEN=60};
private:
	char receive_data[RECIVE_DATA_MAX_LEN];
	int m_data; 			// 实际数据
	bool m_bstate; 		// 当前是否收到数据 (false, true)
private:
	USART_TypeDef * m_usart;
};


#endif


