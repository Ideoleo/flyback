/*
 * ADC_service.hpp
 *
 *  Created on: Jul 23, 2019
 *      Author: MKoza1
 */

#ifndef ADC_SERVICE_HPP_
#define ADC_SERVICE_HPP_
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "cmsis_os.h"

class ADCConf{

public:

	ADCConf();
	~ADCConf();

	float Convert_To_Voltage(uint8_t Vcc_Voltage, uint16_t Max_Bit_Value);
	void ADC_Push(uint32_t Data_To_Push);
	void ADC_Get();

private:

	 osMessageQId ADC_Rx_Handle;
	 uint32_t ADC_Value;

};



#endif /* ADC_SERVICE_HPP_ */
