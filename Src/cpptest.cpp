/*
 * cpptest.cpp
 *
 *  Created on: Jul 16, 2019
 *      Author: MKoza1
 */

#include <vector>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "main.h"
#include "stm32f4xx_it.h"
using namespace std;


 extern UART_HandleTypeDef huart3;
 char MEMDataToSend[128];
 char DataToSend[128];
 char Cmp[4];
 char Str1[] = "Ala";   //String do porownania
 char *WordToFind;
 char MEMWordToFind[32];
 uint16_t size = 0;
 uint8_t i = 0;
 char Data[128];



extern "C" void UART_function(uint8_t ReceivedData) {


	 if (ReceivedData == 13){  //13 - znak entera w ASCII

			DataToSend[i] = 0;
			std::vector <string> vdata;
			strcpy(MEMDataToSend,DataToSend); //MEMDataToSend - pierwsze slowo stringa ||  DataToSend - caly string
			WordToFind = strtok(MEMDataToSend," ");

			while(WordToFind != NULL)
				{

					string temp(WordToFind);
					printf("Word: %s \n\r",WordToFind);
					vdata.push_back (temp);
					WordToFind = strtok(NULL, " ");

				}

			for(int i=0; i < vdata.size(); i++)
				printf("%s \n\r",vdata[i].c_str());


			if(strcmp(MEMDataToSend, Str1) == 0)
				{
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
					strcpy(Cmp, "YES");
				}

			else
				{
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
					strcpy(Cmp, "NO");
				}

			 size = sprintf(Data, "Text: %s  Length: %u  First Word: %s  Compare?: %s \n\n\r",DataToSend,i,WordToFind,Cmp);
			 HAL_UART_Transmit_IT(&huart3, reinterpret_cast<uint8_t*>(Data), size);
			 i = 0;

		 }

		 else
		 {

			 DataToSend[i] = ReceivedData;
			 i++;

		 }



}
