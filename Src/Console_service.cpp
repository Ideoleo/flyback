#include "Console_service.hpp"
#include "PWM_service.hpp"
#include "cmsis_os.h"
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
using namespace std;


UartCom Object1(13); //13 - znak entera w ASCII


UartCom::UartCom(uint8_t Enter_)	//Konstruktor
:Enter(Enter_)
{
	i = 0;

	osMessageQDef(Console_Rx, 16, uint32_t);
	Console_Rx_Handle = osMessageCreate(osMessageQ(Console_Rx), NULL);

}

UartCom::~UartCom(){				//Destruktor

}


void UartCom::UART_Rec_Sign(uint8_t RC_Data){


	osMessagePut(Console_Rx_Handle, RC_Data, 0);

}

void UartCom::UART_Build_String(){


	osEvent evt;
	uint32_t Uart_rec_value;

	evt = osMessageGet(Console_Rx_Handle, osWaitForever);
	Uart_rec_value = evt.value.v;


	 if (Uart_rec_value == Enter){

			 std::vector<std::string> vec_data;		//wektor dla tokenow

			 DataToSend[i] = 0;

			 i = 0;

			vec_data = Object1.UART_Tok(DataToSend, " ");	//podziel na tokeny
			Object1.UART_Class_VPRINT(vec_data);			//wypisz wektor

		 }

		 else{

			 DataToSend[i] = Uart_rec_value;		//wpisz kolejne znaki do bufora
			 i++;

		 }




}

std::vector<std::string> UartCom::UART_Tok(char* MEMDataToSend, const char* const StrFind){

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


	while(1){

		UART_Build_String();

	}

}


/*

void UartCom::UART_Printf(char Txt[50],uint32_t Value){

	static uint8_t data[50];
	uint16_t size = 0;
	UART_HandleTypeDef huart3;

	size = sprintf(data,Txt,Value);
	HAL_UART_Transmit(&huart3, data, size, 10);


}

*/

void UART_Class_RC(uint8_t RC_Data){

	Object1.UART_Rec_Sign(RC_Data);

}

void UART_Class_RUN(void const* param){

	Object1.UART_Class_RUN();

}
