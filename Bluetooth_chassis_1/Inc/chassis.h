/*
 * chassis.h
 *
 *  Created on: Apr 28, 2025
 *      Author: javie
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "stm32f051x8.h"
#include "motor_controller.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	MotorController wheelLeft;
	MotorController wheelRight;
	float safeFactorBackwardsSpeed;  //Range 0 to 1.0 Front
	float maxSpeed;  //Range 0 to 1.0
    float currentAdvanceSpeed;  //Range -1.0 backWards to 1.0 Front
    float currentTurnSpeed;  //Range -1.0 LeftTurn to 1.0 RightTurn
    bool chassisON; //0 = OFF, 1 = ON
    bool advanceInverted; // 0 = normal, 1 = front is at the back
    bool turnInverted; // 0 = normal, 1 = turn directions are inverted
    bool brakeEnabled; // 0 = coastMode, 1 = breakMode
} CHASSIS;

//Init functions
void Init_Chassis(CHASSIS* AGV_Chassis, MotorController wheelLeft, MotorController wheelRight);

//Control functions
void set_FinalSpeedsMotors(CHASSIS* AGV_Chassis);
void set_SafeFactorBackwards(CHASSIS* AGV_Chassis, float safeFactor);
void set_MaxSpeed(CHASSIS* AGV_Chassis, float maxSpeed);
void set_AdvanceSpeed(CHASSIS* AGV_Chassis, float advanceSpeed);
void set_TurnSpeed(CHASSIS* AGV_Chassis, float turnSpeed);

void set_ChassisON(CHASSIS* AGV_Chassis);
void set_ChassisOFF(CHASSIS* AGV_Chassis);
void set_AdvanceInverted(CHASSIS* AGV_Chassis, bool invert);
void set_TurnInverted(CHASSIS* AGV_Chassis, bool invert);
void set_BrakeMode(CHASSIS* AGV_Chassis);
void set_CoastMode(CHASSIS* AGV_Chassis);

//Status functions
float get_CurrentChassisAdvanceSpeed(CHASSIS* AGV_C);
float get_CurrentChassisTurnSpeed(CHASSIS* AGV_C);
bool get_ChassisON(CHASSIS* AGV_C);
bool get_AdvanceInverted(CHASSIS* AGV_C);
bool get_TurnInverted(CHASSIS* AGV_C);
bool get_BrakeEnabled(CHASSIS* AGV_C);

#endif /* CHASSIS_H_ */
