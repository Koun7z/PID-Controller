/*
 * PID.c
 *
 *  Created on: Jan 25, 2025
 *      Author: pwoli
 */
#include "PID.h"

#include <stdlib.h>


static float _clamp(float num, float max, float min)
{
	const float t = num < min ? min : num;
	return t > max ? max : t;
}

bool PID_Init(struct PID_State* regulator, float K, float I, float D)
{
	regulator->K_Gain = K;
	regulator->I_Gain = I;
	regulator->D_Gain = D;

	regulator->_integralState = 0;
	regulator->_previousInput = 0;

	regulator->_previousInputFiltered      = 0;
	regulator->_previousInputBufferPointer = 0;

	regulator->MaxSaturation = __builtin_inff();
	regulator->MinSaturation = -__builtin_inff();

	return true;
}

bool PID_DFilterInit(struct PID_State* regulator, uint16_t dTermFiterOrder, const float* firCoefs)
{
	if(dTermFiterOrder == 0)
	{
		regulator->FIR_Coefficients = NULL;
		return true;
	}
	else if(firCoefs == NULL)
	{
		return false;
	}

	regulator->_previousInputBufferPointer = 0;

	regulator->_previousInputRawBuff = (float*)malloc(dTermFiterOrder * sizeof(float));
	if(regulator->_previousInputRawBuff == NULL)
	{
		return false;
	}

	for(uint16_t i = 0; i < dTermFiterOrder; i++)
	{
		regulator->_previousInputRawBuff[i] = 0;
	}

	return true;
}

float PID_Update(struct PID_State* regulator, float input, float dt)
{
	float output = 0;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5 * regulator->I_Gain * dt;
	regulator->_integralState  = _clamp(regulator->_integralState, regulator->MaxSaturation, regulator->MinSaturation);
	regulator->_previousInput  = input;

	output += regulator->_integralState;

	// Derivative part
	float filteredInput = 0;
	if(regulator->D_FilterOrder > 0)
	{
		for(uint16_t i = 0; i < regulator->D_FilterOrder; i++)
		{
			uint16_t n = (i + regulator->_previousInputBufferPointer) % regulator->D_FilterOrder;

			filteredInput += regulator->_previousInputRawBuff[n] * regulator->FIR_Coefficients[i];
		}
		filteredInput += input * regulator->FIR_Coefficients[regulator->D_FilterOrder];

		regulator->_previousInputBufferPointer = ++(regulator->_previousInputBufferPointer) % regulator->D_FilterOrder;
		regulator->_previousInputRawBuff[regulator->_previousInputBufferPointer] = input;
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
	if(regulator->_previousInputRawBuff != NULL)
	{
		free(regulator->_previousInputRawBuff);
	}
}
