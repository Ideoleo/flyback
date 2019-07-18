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


class UartCom{

public:

	UartCom(uint8_t Enter_);
	~UartCom();

	void UART_Class_RC(uint8_t Data_RC);
	void UART_Class_TOK();
	void UART_Class_VPRINT();

private:

	 std::vector <std::string> v_data;
	 char DataToSend[128],MEMDataToSend[128];
	 uint8_t i;
	 const uint8_t Enter;

};




#endif /* CONSOLE_SERVICE_HPP_ */
