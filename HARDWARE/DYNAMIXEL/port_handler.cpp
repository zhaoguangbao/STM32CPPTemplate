/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include "port_handler.h"

using namespace dynamixel;

PortHandler *PortHandler::getPortHandler(const char *port_name)
{
  //return new PortHandler(port_name);
	return 0;
}

PortHandler::PortHandler(USART_TypeDef* port_name){
	is_using_ = false;
	baudrate_=57600;
	m_usart=port_name;
}

void PortHandler::setPortName(USART_TypeDef* port_name){
	m_usart=port_name;
}

USART_TypeDef* PortHandler::getPortName(){
	return m_usart;
}

bool PortHandler::openPort(){
	uart_init(baudrate_); // 暂时初始化串口1
	return true;
}

void PortHandler::closePort(){
}

void PortHandler::clearPort(){
}
