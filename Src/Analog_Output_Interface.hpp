/*
 * Analog_Output_Interface.hpp
 *
 *  Created on: Jul 25, 2019
 *      Author: MKoza1
 */

#ifndef ANALOG_OUTPUT_INTERFACE_HPP_
#define ANALOG_OUTPUT_INTERFACE_HPP_

#include <cstdint>


class AnalogOutInterface{

public:

	virtual void Set_Out(uint32_t value) = 0;


private:



};


#endif /* ANALOG_OUTPUT_INTERFACE_HPP_ */
