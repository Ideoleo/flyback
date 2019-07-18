#include "Console_service.hpp"
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

		 DataToSend[i] = 0;
		 i = 0;
		 strcpy(MEMDataToSend,DataToSend);

	 }

	 else{

		 DataToSend[i] = RC_Data;
		 i++;

	 }

}

void UartCom::UART_Class_TOK(){


	char *WordToFind;
	WordToFind = strtok(MEMDataToSend," ");

	while(WordToFind != NULL){

		string temp(WordToFind);
		v_data.push_back (temp);
		WordToFind = strtok(NULL, " ");

	}


}

void UartCom::UART_Class_VPRINT(){

	for(int i = 0; i < v_data.size(); i++){

			printf("%s \n\r",v_data[i].c_str());
		}

}

extern "C" void UART_Class_RC(uint8_t RC_Data){

	Object1.UART_Class_RC(RC_Data);
	Object1.UART_Class_TOK();
	Object1.UART_Class_VPRINT();

}
