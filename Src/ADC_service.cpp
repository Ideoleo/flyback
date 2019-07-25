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


ADCConf ADC_Ob1;


ADCConf::ADCConf(){

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


/*

void ADCConf::ADC_Value_Send(){
                               	   	   	   	   //Tu wpisac funkcje w piatek
}

*/

 /*

 void ADCConf::Change_PWM(){

	 TIM_HandleTypeDef htim4;
	 htim4.Instance = TIM4;

	 uint8_t PWM_Control = ADC_Ob1.Convert_To_PWM();			//Skalowanie wartosci ADC od 0 do 100
	 htim4.Instance -> CCR3 = PWM_Control;


 }
 */


void ADC_Transmit(uint32_t ADC_Value){

	ADC_Ob1.ADC_Push(ADC_Value);

}


///////////////////////////////////////////////////////////////////////////////////////////////////

void ADC_Print(void const* param){

	uint32_t cnt = 0;


	while(1){

		ADC_Ob1.ADC_Get();

		if(cnt%20 == 0){

			printf("ADC: %.2f V \n\r",(ADC_Ob1.Convert_To_Voltage()));

		}

		cnt++;

	}


}
//////////////////////////////////////////////////////////////////////////////////////////////////////



