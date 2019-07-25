/*
 * PWM_service.cpp
 *
 *  Created on: Jul 25, 2019
 *      Author: MKoza1
 */

#include "PWM_service.hpp"


PWMConf::PWMConf(TIM_HandleTypeDef* const tim)
: tim(tim){



}

PWMConf::~PWMConf(){



}

uint32_t PWMConf::Convert_To_PWM(uint32_t value){


	return ((102*value)/Max_Bit_Value); //zmiana wypelnienia w zakresie od 0 - 100 %

}

void PWMConf::Set_Out(uint32_t value){

	uint32_t PWM_Value = Convert_To_PWM(value);
	tim->Instance->CCR3 = PWM_Value;

}








