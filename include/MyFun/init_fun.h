/*
 * init_fun.h
 *
 *  Created on: August 12, 2013
 *      Author: Max
 */

void InitRCC(void);
void GPIO_Conf_AllToAIN(void);
void InitNVIC(void);
void InitSwitch(void);
void InitADC(void);
void InitDMA(void);
void InitUSART(void);
void InitPWMMotorInterface(void);
void InitPulseCounter(void);
void InitAbsoluteEncoderInterface();
void InitIncrementalEncoderInterface();
void InitAll(void);
