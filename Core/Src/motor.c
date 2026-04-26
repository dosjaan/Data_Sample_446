/*
 * motor.c
 *
 *  Created on: Jan 2, 2025
 *      Author: doska
 */
#include "motor.h"
#include "math.h"
#include <stdint.h> // Include for standard integer types like uint16_t, int16_t
#include <stdlib.h> // Include for abs()
double ML;
double MR;
double speed;
double angle;
double pi=3.14159265359;


float SATURATE(float _IN, float _MIN, float _MAX) {
    if (_IN < _MIN) return _MIN;
    if (_IN > _MAX) return _MAX;
    return _IN;
}
void Estimate_Speed(Motor_Encoder_Variables *Encoder, uint16_t SamplingTimeMs) {
	if (SamplingTimeMs == 0) return;


	// Update current position
	Encoder->CurrentPos = Encoder->timer_address->Instance->CNT;
	Encoder->difference = Encoder->CurrentPos - Encoder->Oldpos;

	// Convert difference to speed
	Encoder->Speed = (float)Encoder->difference / SamplingTimeMs;

	// Apply exponential filtering
//	Encoder->Speed = Alpha * Encoder->Speed + (1.0f - Alpha) * Encoder->PreviousFilteredSpeed;

	// Update moving average filter
	Encoder->SpeedHistory[Encoder->SpeedIndex] = Encoder->Speed;
	Encoder->SpeedIndex = (Encoder->SpeedIndex + 1) % MA_FILTER_SIZE;

	// Calculate moving average
	float sum = 0.0f;
	for (uint8_t i = 0; i < MA_FILTER_SIZE; i++) {
		sum += Encoder->SpeedHistory[i];
	}
	Encoder->Speed =(int16_t)( sum / MA_FILTER_SIZE);

	// Update previous filtered speed and position
	Encoder->PreviousFilteredSpeed = Encoder->Speed;
	Encoder->Oldpos = Encoder->CurrentPos;
}


void PID_init(PID_Variables *PID, float KP, float KI, float KD, int16_t S_MIN, int16_t S_MAX, int16_t CO_MIN, int16_t CO_MAX) {
    PID->KP = KP;
    PID->KI = KI;
    PID->KD = KD;
    PID->SP_MAX = S_MAX;
    PID->SP_MIN = S_MIN;
    PID->CO_MIN = CO_MIN;
    PID->CO_MAX = CO_MAX;
    PID->I_error = 0.0f;
    PID->lastError = 0.0f;
    PID->CO = 0.0f;
    PID->D_error = 0.0f;
}

void PID_estimate(PID_Variables *PID) {
	PID->SP=SATURATE(PID->SP, PID->SP_MIN, PID->SP_MAX);
	// Calculate error
	PID->error = PID->SP - PID->PV;
	PID->I_error += PID->error;
	PID->I_error=SATURATE(PID->I_error, PID->CO_MIN, PID->CO_MAX);
	float D_term = PID->error - PID->lastError;
	PID->D_error = 0.9f * PID->D_error + 0.1f * D_term; // Adjustable low-pass filter
	// Compute control output
	PID->CO = (PID->KP * PID->error) + (PID->KI * PID->I_error) + (PID->KD * PID->D_error);
	// Saturate control output
	PID->CO=SATURATE(PID->CO, PID->CO_MIN, PID->CO_MAX);
	// Store current error for next derivative calculation
	PID->lastError = PID->error;
}



float map(float x, float in_min, float in_max, float out_min, float out_max) {
	// Check for invalid input range
	if (in_min >= in_max) {
		return 0.0f; // Handle error
	}
	// Ensure x is within bounds
	if (x < in_min) x = in_min;
	if (x > in_max) x = in_max;
	// Map input to output range
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DC_MOTOR_SET_DIRECTION(DC_Motor_GPIO_t *Motor,int16_t speed)
{
	if(speed>0)
	{
		*(Motor->PWM_CHANNEL_ADDRESS)=abs(speed);
		Motor->portA->BSRR = Motor->pinA;                         // Set GPIO_A high
		Motor->portB->BSRR = (uint32_t)Motor->pinB << 16;         // Set GPIO_B low
	}
	else if(speed<0)
	{
		// Set GPIO_A low and GPIO_B high for backward direction
		*(Motor->PWM_CHANNEL_ADDRESS)=abs(speed);
		Motor->portA->BSRR = (uint32_t)Motor->pinA << 16;         // Set GPIO_A low
		Motor->portB->BSRR = Motor->pinB;   // Set GPIO_B high
	}
	else
	{
		Motor->portA->BSRR = (uint32_t)Motor->pinA << 16;         // Set GPIO_A low
		Motor->portB->BSRR = (uint32_t)Motor->pinB << 16;         // Set GPIO_B low
	}
}


