/*
 * ADC_service.cpp
 *
 *  Created on: Jul 23, 2019
 *      Author: MKoza1
 */


#include "ADC_service.hpp"
#include "main.h"
#include <fstream>
#include <iostream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
using namespace std;



ADCConf::ADCConf(AnalogOutInterface* const wsk)
:wsk(wsk){

	Vcc_Voltage = 3;
	Max_Bit_Value = 4096;   //2^12
	osMessageQDef(ADC_Rx, 16, uint32_t);
	ADC_Rx_Handle = osMessageCreate(osMessageQ(ADC_Rx), NULL);

}

ADCConf::~ADCConf(){


}

void ADCConf::ADC_Push(uint32_t Data_To_Push){

	osMessagePut(ADC_Rx_Handle, Data_To_Push, 0);
}

void ADCConf::ADC_Get(){

	osEvent event;

	event = osMessageGet(ADC_Rx_Handle, osWaitForever);
	ADC_Value = event.value.v;

}


float ADCConf::Convert_To_Voltage(){


	float Voltage_ADC = (((float)ADC_Value/Max_Bit_Value)*Vcc_Voltage);
	return Voltage_ADC;

}


float ADCConf::ADC_Send_Voltage(){

	ADC_Get();
	return(Convert_To_Voltage());

}


void ADCConf::ADC_Send_PWM(){

	 for(;;)
	  {
		ADC_Get();
		wsk -> Set_Out(ADC_Value);

	  }



}






