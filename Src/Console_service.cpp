#include "Console_service.hpp"
#include "cmsis_os.h"
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
using namespace std;



UartCom Object1(13); //13 - znak entera w ASCII

UartCom::UartCom(uint8_t Enter_)
:Enter(Enter_)
{
	i = 0;
}

UartCom::~UartCom(){

}


void UartCom::UART_Class_RC(uint8_t RC_Data){



	 if (RC_Data == Enter){

		 std::vector<std::string> vec_data;

		 DataToSend[i] = 0;
		 i = 0;

		 vec_data = Object1.UART_Class_TOK(DataToSend, " ");
		 Object1.UART_Class_VPRINT(vec_data);

	 }

	 else{

		 DataToSend[i] = RC_Data;
		 i++;

	 }

}

std::vector<std::string> UartCom::UART_Class_TOK(char* MEMDataToSend, const char* const StrFind){

	std::vector<std::string> vdata;
	char *WordToFind;
	WordToFind = strtok(MEMDataToSend,StrFind);


	while(WordToFind != NULL){

		string temp(WordToFind);
		vdata.push_back (temp);
		WordToFind = strtok(NULL,StrFind);

	}

	return vdata;


}

void UartCom::UART_Class_VPRINT(std::vector<std::string> vdata){

	for(unsigned int i = 0; i < vdata.size(); i++){

			printf("%s \n\r",vdata[i].c_str());

		}

}

void UartCom::UART_Class_RUN(){

	int i = 0;
	while(1){


		osDelay(1000);
		printf("Test: %d\n\r",i);
		i++;

	}


}


extern "C" void UART_Class_RC(uint8_t RC_Data){

	Object1.UART_Class_RC(RC_Data);

}

extern "C" void UART_Class_RUN(void const* param){

	Object1.UART_Class_RUN();

}
