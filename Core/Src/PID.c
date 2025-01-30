/*
 * PID.c
 *
 *  Created on: Jan 25, 2025
 *      Author: pwoli
 */
#include "PID.h"

#include <stdlib.h>


float _clamp(float num, float min, float max)
{
	const float t = num < min ? min : num;
	return t > max ? max : t;
}

bool PID_Innit(struct PID_State* regulator, float K, float I, float D, uint16_t dTermFiterOrder, const float* firCoefs)
{
	regulator->K_Gain = K;
	regulator->I_Gain = I;
	regulator->D_Gain = D;

	regulator->_integralState              = 0;
	regulator->_previousInputFiltered      = 0;
	regulator->_previousInput              = 0;
	regulator->_previousInputBufferPointer = 0;

	regulator->MaxSaturation = __builtin_inff();
	regulator->MinSaturation = -__builtin_inff();

	if(dTermFiterOrder == 0)
	{
		regulator->FIR_Coefficients = NULL;
		return true;
	}
	else if(firCoefs == NULL)
	{
		return false;
	}

	regulator->_previousInputRaw = (float*)malloc(dTermFiterOrder * sizeof(float));
	if(regulator->_previousInputRaw == NULL)
	{
		return false;
	}

	for(uint16_t i = 0; i < dTermFiterOrder; i++)
	{
		regulator->_previousInputRaw[i] = 0;
	}

	return true;
}

float PID_Update(struct PID_State* regulator, float input, float dt)
{
	float output = 0;

	output += input * regulator->K_Gain;

	regulator->_integralState += (input + regulator->_previousInput) * 0.5 * dt;
	regulator->_integralState  = _clamp(regulator->_integralState, regulator->MaxSaturation, regulator->MinSaturation);

	regulator->_previousInput = input;

	output += regulator->_integralState * regulator->I_Gain;

	float filteredInput = 0;
	if(regulator->D_FilterOrder > 0)
	{
		for(uint16_t i = 0; i < regulator->D_FilterOrder; i++)
		{
			uint16_t n = (i + regulator->_previousInputBufferPointer) % regulator->D_FilterOrder;

			filteredInput += regulator->_previousInputRaw[n] * regulator->FIR_Coefficients[i];
		}
		filteredInput += input * regulator->FIR_Coefficients[regulator->D_FilterOrder];

		regulator->_previousInputBufferPointer = ++(regulator->_previousInputBufferPointer) % regulator->D_FilterOrder;
		regulator->_previousInputRaw[regulator->_previousInputBufferPointer] = input;
	}
	else
	{
		filteredInput = input;
	}

	output += (filteredInput - regulator->_previousInputFiltered) / dt;

	regulator->_previousInputFiltered = filteredInput;

	return _clamp(output, regulator->MaxSaturation, regulator->MinSaturation);
}


void PID_Destroy(struct PID_State* regulator)
{
	if(regulator->_previousInputRaw != NULL)
	{
		free(regulator->_previousInputRaw);
	}
}
