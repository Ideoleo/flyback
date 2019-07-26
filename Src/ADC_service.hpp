/*
 * ADC_service.hpp
 *
 *  Created on: Jul 23, 2019
 *      Author: MKoza1
 */

#ifndef ADC_SERVICE_HPP_
#define ADC_SERVICE_HPP_
#include "main.h"
#include <fstream>
#include <iostream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "cmsis_os.h"
#include "Analog_Output_Interface.hpp"

class ADCConf{

public:

	ADCConf(AnalogOutInterface* const wsk);
	~ADCConf();

	float Convert_To_Voltage();
	void ADC_Push(uint32_t Data_To_Push);
	uint8_t Convert_To_PWM();
	void ADC_Get();
	void Change_PWM();
	float ADC_Send_Voltage();
	void ADC_Send_PWM();


private:

	 osMessageQId ADC_Rx_Handle;
	 uint32_t ADC_Value;
	 uint8_t Vcc_Voltage;
	 uint16_t Max_Bit_Value;
	 AnalogOutInterface* const wsk;



};



#endif /* ADC_SERVICE_HPP_ */
