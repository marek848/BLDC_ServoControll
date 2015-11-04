/*
 * main_fun.h
 *
 *  Created on: August 12, 2013
 *      Author: Max
 */

uint8_t SendDataByUSART(char *str, uint16_t packetSize);

void Delay(uint32_t nTime);

void TimingDelay_Decrement(void);

void WatchDog_Decrement(void);

void SwitchReader(void);

void ForceConvertion(void);

void PositionConvertion(void);

void ActionCalibrateForce(void);

void ActionCalibratePosition(void);

void MotorOffHighImp(void);

void MotorOffLowImp(void);

void MotorDirectControl(void);

void MotorForceCalculation(void);

void MotorForceControl(void);

void MotorSpeedControl(void);

void MotorPositionCalculation(void);

void MotorPositionControl(void);

void MotorFuzzyControl(void);

void StateTabInterpreter(void);

