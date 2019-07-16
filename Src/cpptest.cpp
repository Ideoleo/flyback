/*
 * cpptest.cpp
 *
 *  Created on: Jul 16, 2019
 *      Author: MKoza1
 */

#include <vector>

extern "C" void UART_function() {


	/*
	 static char Data[128];
	 char MEMDataToSend[128];
	 int size = 0;

		 if (ReceivedData == 13){  //13 - znak entera w ASCII
			DataToSend[i] = 0;

			strcpy(MEMDataToSend,DataToSend);

		    WordToFind = strtok(MEMDataToSend," ");

		    if(strcmp(WordToFind, Str1) == 0){
		            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		    		strcpy(Cmp, "YES");
		    }
		    else{
		    		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		    		strcpy(Cmp, "NO");
		    }

		     size = sprintf(Data, "Text: %s  Length: %u  First Word: %s  Compare?: %s \n\n\r", DataToSend,i,WordToFind, Cmp);
			 HAL_UART_Transmit_IT(&huart3, Data, size);
			 i = 0;
			 test();

		 }
		 else{

			 DataToSend[i] = ReceivedData;
			 i++;

		 }

		 HAL_UART_Receive_IT(&huart3, &ReceivedData, 1);

		 */

	std::vector<int> a;
}
