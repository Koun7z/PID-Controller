/*
 * PID.h
 *
 *  Created on: Jan 25, 2025
 *      Author: pwoli
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdbool.h>
#include <stdint.h>

struct PID_State
{
	// PID Config
	float K_Gain;
	float I_Gain;
	float D_Gain;
	float MaxSaturation;
	float MinSaturation;

	// Filter Config
	uint16_t D_FilterOrder;
	float* FIR_Coefficients;

	// PID State
	float _integralState;
	float _previousInput;

	// Filter State
	float* _previousInputRaw;
	uint16_t _previousInputBufferPointer;
};

/*
 * @brief Initializes PID internal structure
 * @param[out] *regulator - pointer to pid struct to initialize
 * @param[in]   K - proportional gain
 * @param[in]   I - integral gain
 * @param[in]   D - differential gain
 * @param[in]   dTermFiterOrder - order of differential filtering (0 for no filtering)
 * @param[in]  *firCoefs - pointer to an array of FIR coefficients,
 * 			    array size = dTermFiterOrder + 1,
 * 			    can be null if filter is off
 * @return 	    bool
 * 			    - true initialization complete
 * 			    - initialization failed
 */
bool PID_Innit(struct PID_State* regulator, float K, float I, float D, uint16_t dTermFiterOrder, const float* firCoefs);

/*
 * @brief Calculates PID response based on given input and updates internal regulator state
 * @param[in] *regulator - pointer to pid struct to update
 * @param[in]  input - controll error
 * @param[in]  dt - sampling period (time beetwen two update calls)
 * @return 	   float
 * 			   - PID output
 */
float PID_Update(struct PID_State* regulator, float input, float dt);

/*
 * @brief Deallocates all dynamiclly allocated memory
 * @param[in] *regulator - pointer to pid struct to update
 * @return 	   void
 */
void PID_Destroy(struct PID_State* regulator);
#endif /* INC_PID_H_ */
