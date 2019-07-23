/*
 * ADC_service.cpp
 *
 *  Created on: Jul 23, 2019
 *      Author: MKoza1
 */


#include "ADC_service.hpp"
#include "main.h"
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstring>
using namespace std;


ADCConf ADC_Ob1;


ADCConf::ADCConf(){

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


float ADCConf::Convert_To_Voltage(uint8_t Vcc_Voltage, uint16_t Max_Bit_Value){


	float Voltage_ADC = (((float)ADC_Value/Max_Bit_Value)*Vcc_Voltage);

	return Voltage_ADC;

}


extern "C" void ADC_Transmit(uint32_t ADC_Value){


		ADC_Ob1.ADC_Push(ADC_Value);


}


