/*
 * PID.c
 *
 *  Created on: Jan 25, 2025
 *      Author: Piotr Woliński
 */
#include "PID.h"

#include <stdlib.h>
#include <math.h>

static float _clampf32(float num, float min, float max)
{
	const float t = num < min ? min : num;
	return t > max ? max : t;
}

static double _clampf64(double num, double min, double max)
{
	const double t = num < min ? min : num;
	return t > max ? max : t;
}

void PID_Init_f32(PID_Instance_f32* regulator, float K, float I, float D)
{
	regulator->K_Gain = K;
	regulator->I_Gain = I;
	regulator->D_Gain = D;

	regulator->_integralState = 0.0f;
	regulator->_previousInput = 0.0f;
	regulator->_previousInputFiltered = 0.0f;

	regulator->MaxOutput = INFINITY;
	regulator->MinOutput = -INFINITY;

	regulator->MaxWindup = INFINITY;
	regulator->MinWindup = -INFINITY;
}

void PID_SetSaturation_f32(PID_Instance_f32* regulator, float minOutput, float maxOutput)
{
	regulator->MinOutput = minOutput;
	regulator->MaxOutput = maxOutput;
}

void PID_SetAntiWindup_f32(PID_Instance_f32* regulator, float minWindup, float maxWindup)
{
	regulator->MinWindup = minWindup;
	regulator->MaxWindup = maxWindup;
}

float PID_Update_f32(PID_Instance_f32* regulator, float input, float dt)
{
	float output = 0.0f;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5f * regulator->I_Gain * dt;

#if ANTI_WINDUP_ENABLE
	regulator->_integralState  = _clampf32(regulator->_integralState, regulator->MinWindup, regulator->MaxWindup);
#endif

	// Differential part
	output += (input - regulator->_previousInput) * dt * regulator->D_Gain;
	regulator->_previousInput = input;

#if OUTPUT_SATURATION_ENABLE
	output = _clampf32(output, regulator->MinOutput, regulator->MaxOutput);
#endif

	return output;
}

float PID_DTermFIR_Update_f32(PID_Instance_f32* regulator, DSP_FIR_RT_Instance_f32* filter, float input, float dt)
{
	float output = 0.0f;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5f * regulator->I_Gain * dt;

#if ANTI_WINDUP_ENABLE
	regulator->_integralState = _clampf32(regulator->_integralState, regulator->MinWindup, regulator->MaxWindup);
#endif

	// Differential part
	const float filteredInput = DSP_FIR_RT_Update_f32(filter, input);

	output += (filteredInput - regulator->_previousInputFiltered) * dt * regulator->D_Gain;
	regulator->_previousInput = input;
	regulator->_previousInputFiltered = filteredInput;

#if OUTPUT_SATURATION_ENABLE
	output = _clampf32(output, regulator->MinOutput, regulator->MaxOutput);
#endif

	return output;
}

float PID_DTermIIR_Update_f32(PID_Instance_f32* regulator, DSP_IIR_RT_Instance_f32* filter, float input, float dt)
{
	float output = 0.0f;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5f * regulator->I_Gain * dt;

#if ANTI_WINDUP_ENABLE
	regulator->_integralState = _clampf32(regulator->_integralState, regulator->MinWindup, regulator->MaxWindup);
#endif

	// Differential part
	const float filteredInput = DSP_IIR_RT_Update_f32(filter, input);

	output += (filteredInput - regulator->_previousInputFiltered) * dt * regulator->D_Gain;
	regulator->_previousInput = input;
	regulator->_previousInputFiltered = filteredInput;

#if OUTPUT_SATURATION_ENABLE
	output = _clampf32(output, regulator->MinOutput, regulator->MaxOutput);
#endif

	return output;
}

void PID_Init_f64(PID_Instance_f64* regulator, double K, double I, double D)
{
	regulator->K_Gain = K;
	regulator->I_Gain = I;
	regulator->D_Gain = D;

	regulator->_integralState = 0;
	regulator->_previousInput = 0;
	regulator->_previousInputFiltered = 0;

	regulator->MaxOutput = INFINITY;
	regulator->MinOutput = -INFINITY;

	regulator->MaxWindup = INFINITY;
	regulator->MinWindup = -INFINITY;
}

void PID_SetSaturation_f64(PID_Instance_f64* regulator, double minOutput, double maxOutput)
{
	regulator->MinOutput = minOutput;
	regulator->MaxOutput = maxOutput;
}

void PID_SetAntiWindup_f64(PID_Instance_f64* regulator, double minWindup, double maxWindup)
{
	regulator->MinWindup = minWindup;
	regulator->MaxWindup = maxWindup;
}

double PID_Update_f64(PID_Instance_f64* regulator, double input, double dt)
{
	double output = 0.0;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5 * regulator->I_Gain * dt;

#if ANTI_WINDUP_ENABLE
	regulator->_integralState = _clampf64(regulator->_integralState, regulator->MinWindup, regulator->MaxWindup);
#endif

	// Differential part
	output += (input - regulator->_previousInput) * dt * regulator->D_Gain;
	regulator->_previousInput = input;

#if OUTPUT_SATURATION_ENABLE
	output = _clampf64(output, regulator->MinOutput, regulator->MaxOutput);
#endif

	return output;
}

double PID_DTermFIR_Update_f64(PID_Instance_f64* regulator, DSP_FIR_RT_Instance_f64* filter, double input, double dt)
{
	double output = 0.0;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5f * regulator->I_Gain * dt;

#if ANTI_WINDUP_ENABLE
	regulator->_integralState = _clampf64(regulator->_integralState, regulator->MinWindup, regulator->MaxWindup);
#endif

	// Differential part
	const double filteredInput = DSP_FIR_RT_Update_f64(filter, input);

	output += (filteredInput - regulator->_previousInputFiltered) * dt * regulator->D_Gain;
	regulator->_previousInput = input;
	regulator->_previousInputFiltered = filteredInput;

#if OUTPUT_SATURATION_ENABLE
	output = _clampf64(output, regulator->MinOutput, regulator->MaxOutput);
#endif

	return output;
}

double PID_DTermIIR_Update_f64(PID_Instance_f64* regulator, DSP_IIR_RT_Instance_f64* filter, double input, double dt)
{
	double output = 0.0;

	// Proportional part
	output += input * regulator->K_Gain;

	// Integral part
	regulator->_integralState += (input + regulator->_previousInput) * 0.5 * regulator->I_Gain * dt;

#if ANTI_WINDUP_ENABLE
	regulator->_integralState = _clampf64(regulator->_integralState, regulator->MinWindup, regulator->MaxWindup);
#endif

	// Differential part
	const double filteredInput = DSP_IIR_RT_Update_f64(filter, input);

	output += (filteredInput - regulator->_previousInputFiltered) * dt * regulator->D_Gain;
	regulator->_previousInput = input;
	regulator->_previousInputFiltered = filteredInput;

#if OUTPUT_SATURATION_ENABLE
	output = _clampf64(output, regulator->MinOutput, regulator->MaxOutput);
#endif

	return output;
}

