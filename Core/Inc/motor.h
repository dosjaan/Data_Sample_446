/*
 * motor.h
 *
 *  Created on: Jan 2, 2025
 *      Author: doska
 */
#include "main.h"
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#define BLDC_Init_Speed 48
#define BLDC_MAX_SPEED 120
#define BLDC_MIN_SPEED 54
#define BLDC_STEP 8
#define BLDC_BRAKE 45
#define ROBOMASTER_MAX_SPEED 10000
#define ROBOMASTER_MIN_SPEED 0
#define MA_FILTER_SIZE 10
#define Alpha 0.8

#define MAX_CAN_RM		 8
#define NUM_OF_CAN		 1
#define RM_TX_GRP1_ID	 0x200
#define RM_TX_GRP2_ID	 0x1FF
#define CAN_DATA_SIZE	 8
#define CAN1_RX_ID_START 0x201

typedef struct
{
	float KP;
	float KI;
	float KD;
	float error;
	float I_error;
	float D_error;
	float lastError;
	float SP; 	/*SetPoint*/
	float PV; 	/*ProcessVariable*/
	int16_t CO; 	/*ControlOutput*/
	int16_t SP_MAX;
	int16_t SP_MIN;
	int16_t CO_MAX;
	int16_t CO_MIN;


} PID_Variables;

typedef struct {
    int16_t Oldpos;                // Previous position (signed for direction handling)
    int16_t CurrentPos;            // Current position
    int16_t difference;            // Difference between positions (signed)
    float Speed;                   // Calculated speed (signed)
    TIM_HandleTypeDef *timer_address; // Pointer to timer structure
    float SpeedHistory[MA_FILTER_SIZE]; // Moving average filter data
    uint8_t SpeedIndex;            // Current index in speed history
    float PreviousFilteredSpeed;   // Last filtered speed value
} Motor_Encoder_Variables;

typedef enum {
	CAN1_MOTOR0,
	CAN1_MOTOR1,
	CAN1_MOTOR2,
	CAN1_MOTOR3,
	CAN1_MOTOR4,
	CAN1_MOTOR5,
	CAN1_MOTOR6,
	CAN1_MOTOR7,
	MAX_NUM_OF_MOTORS
} Motor;

typedef enum {
	CAN_GROUP_ID = 0x200,
	CAN_3508_M1_ID,
	CAN_3508_M2_ID,
	CAN_3508_M3_ID,
	CAN_3508_M4_ID,
	CAN_3508_M5_ID,
	CAN_3508_M6_ID,
	CAN_3508_M7_ID,
	CAN_3508_M8_ID,
} can_msg_id_e;
typedef struct{
	double LX;
	double LY;
	double RX;
	double RY;
	double X;
	double Y;
}joystick_variables;
typedef struct {
    GPIO_TypeDef *portA;
    uint16_t pinA;
    GPIO_TypeDef *portB;
    uint16_t pinB;
    uint32_t *PWM_CHANNEL_ADDRESS;
} DC_Motor_GPIO_t;
void Estimate_Speed(Motor_Encoder_Variables *Encoder, uint16_t SamplingTimeMs);
void PID_estimate(PID_Variables *PID);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void PID_init(PID_Variables *PID, float KP , float KI, float KD,int16_t S_MAX, int16_t S_MIN,int16_t P_MAX,int16_t P_MIN);
void Estimate_Setpoint_BRM(joystick_variables *joy , PID_Variables *Motor[]);
void Lift_Motor(int mot_out);
void DC_MOTOR_SET_DIRECTION(DC_Motor_GPIO_t *Motor,int16_t speed );
float SATURATE(float _IN, float _MIN, float _MAX);
#endif /* INC_MOTOR_H_ */
