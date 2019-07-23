/*
 * Console_service.hpp
 *
 *  Created on: Jul 17, 2019
 *      Author: MKoza1
 */

#ifndef CONSOLE_SERVICE_HPP_
#define CONSOLE_SERVICE_HPP_
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "cmsis_os.h"


class UartCom{

public:

	UartCom(uint8_t Enter_);
	~UartCom();

	void UART_Rec_Sign(uint8_t Data_RC);
	void UART_Build_String();
	std::vector<std::string> UART_Tok(char* MEMDataToSend,const char* const StrFind);
	void UART_Class_VPRINT(std::vector<std::string> vdata);
	void UART_Class_RUN();

private:

	 char DataToSend[128];
	 uint8_t i;
	 const uint8_t Enter;
	 osMessageQId Console_Rx_Handle;


};




#endif /* CONSOLE_SERVICE_HPP_ */
