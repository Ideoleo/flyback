/*
 * PWM_service.hpp
 *
 *  Created on: Jul 25, 2019
 *      Author: MKoza1
 */

#ifndef PWM_SERVICE_HPP_
#define PWM_SERVICE_HPP_

#include "Analog_Output_Interface.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

class PWMConf:public AnalogOutInterface{

public:

	PWMConf(TIM_HandleTypeDef* const tim);
	~PWMConf();
	virtual void Set_Out(uint32_t value);


private:

	static const uint16_t Max_Bit_Value = 4096;
	uint32_t Convert_To_PWM(uint32_t value);
	TIM_HandleTypeDef* const tim;



};



#endif /* PWM_SERVICE_HPP_ */
