/*
 * main.h
 *
 *  Created on: Aug 16, 2012
 *      Author: Max
 */
#ifndef __main_H
#define __main_H

// Includes // ============================================================================
#include "main.h"

#include "stm32f10x.h"
#include "handlers.h"
#include "init_fun.h"
#include "main_fun.h"
#include <string.h> //obs³uga memmove()

volatile uint32_t ZeroPos,Frequency,Amplitude;
extern int16_t sinTab[360];

volatile uint16_t czas1,czas2;
volatile uint32_t timingDelay;

// comment or uncomment
//#define MANIPULATOR_DEVICE

#define DEVICE_ID 					1

#define DELTA_T 1000 //in micro second

// CINTROLLER DEF AND DATA // ==============================================================

#define ADD_ACTION					stateTab[7]
#define ACTION_NOTHING				0
#define ACTION_CALIBRATE_FORCE		1
#define ACTION_CALIBRATE_POSITION	2

// MOTOR CONTROL DATA // ===================================================================

#define WATCH_DOG stateTab[12]



#define MOTOR_MODE					stateTab[9]
#define MOTOR_OFF_HIGH_IMP			0
#define MOTOR_OFF_LOW_IMP			1
#define MOTOR_DIRECT_CONTROL		2
#define MOTOR_FORCE_CONTROL			4
#define MOTOR_SPEED_CONTROL			8
#define MOTOR_POSITION_CONTROL		16
#define MOTOR_FUZZY_CONTROL 		32

// PID STRUCTURE //=========================================================================
// 0 - nastawy regulatora proporcjonalnego
// 1 - nastawy regulatora ca³kuj¹cego
// 2 - nastawy regulatora ró¿niczkuj¹cego
struct PID{
	uint16_t num[3];
	uint16_t den[3];
};

// PID FORCE // ============================================================================

extern volatile int32_t pidTabForce[4];

#define CTRL_VALUE_FORCE					ST_TARGET_FORCE				// Controlled value
#define FEEDBACK_VALUE_FORCE				ST_ACTUAL_FORCE				//Feedback value
#define ERROR_VALUE_FORCE			(int32_t)(CTRL_VALUE_FORCE - FEEDBACK_VALUE_FORCE)/708
#define OUTPUT_DIR_FORCE			pidTabForce[0]					// Output of PID
#define OUTPUT_P_FORCE				pidTabForce[1]					// Output of P
#define OUTPUT_I_FORCE				pidTabForce[2]					// Output of I
#define OUTPUT_D_FORCE				pidTabForce[3]					// Output of D

extern struct PID PID_FORCE;

#define DEL_T_FORCE DELTA_T
//
//#define K_P_FORCE_NUM				3
//#define K_P_FORCE_DEN				3
//
//#define T_I_FORCE_NUM				0//9
//#define T_I_FORCE_DEN				100000
//
//#define T_D_FORCE_NUM				0//10000
//#define T_D_FORCE_DEN				5

extern volatile int32_t integMemForce;
#define INTEG_WINDUP_LIM_FORCE		1000000

extern volatile int32_t derivSumForce;
extern volatile int32_t derivSumForcePast;
extern volatile int32_t derivIterForce;
#define DERIV_MEM_SIZE 10
extern volatile int32_t derivMemForce[DERIV_MEM_SIZE];

// PID POSITION // ============================================================================

extern volatile int32_t pidTabPosition[4];

#define CTRL_VALUE_POSITION					ST_TARGET_POSITION				// Controlled value
#define FEEDBACK_VALUE_POSITION				ST_ACTUAL_POSITION_OUT				//Feedback value
#define ERROR_VALUE_POSITION			(int32_t)(CTRL_VALUE_POSITION - FEEDBACK_VALUE_POSITION)
#define OUTPUT_DIR_POSITION				pidTabPosition[0]					// Output of PID
#define OUTPUT_P_POSITION				pidTabPosition[1]					// Output of P
#define OUTPUT_I_POSITION				pidTabPosition[2]					// Output of I
#define OUTPUT_D_POSITION				pidTabPosition[3]					// Output of D

extern struct PID PID_POSITION;

#define DEL_T_POSITION DELTA_T
//
//#define K_P_POSITION_NUM				6
//#define K_P_POSITION_DEN				50
//
//#define T_I_POSITION_NUM				0
//#define T_I_POSITION_DEN				100000
//
//#define T_D_POSITION_NUM				0
//#define T_D_POSITION_DEN				5

extern volatile int32_t integMemPosition;
#define INTEG_WINDUP_LIM_POSITION		1000000

extern volatile int32_t derivSumPosition;
extern volatile int32_t derivSumPositionPast;
extern volatile int32_t derivIterPosition;
#define DERIV_MEM_POS_SIZE 				1
extern volatile int32_t derivMemPosition[DERIV_MEM_POS_SIZE];

// ADC // ==================================================================================
#define ADC_POTENCJOMETER_1		ADC_Channel_11
#define ADC_POTENCJOMETER_2		ADC_Channel_12
#define ADC_LOAD_CELL			ADC_Channel_2
#define ADC_CURRENT		 		ADC_Channel_1


#define CURRENT_VALUE				adcTabAverage[0]
#define LOAD_CELL_VALUE				adcTabAverage[1]
#define POTENCJOMETER_1_VALUE		adcTabAverage[2]

#define ADC_AVERAGE_COUNT	20						// Number of samples that are
#define ADC_TAB_SIZE		3
//volatile uint16_t adcTemp[ADC_AVERAGE_COUNT]; // Tablica z histori¹ pomiarów
volatile uint16_t adcTab[ ADC_TAB_SIZE ];			// Contain actual non averaged value from adc.
volatile uint16_t adcTabPart[ ADC_TAB_SIZE ];		// Temporary array to create averaged values.
volatile uint16_t adcTabAverage[ ADC_TAB_SIZE ];	// Array of averaged values.
volatile uint16_t adcCount;


// USART // ===============================================================================

// Data packet should look as follows:
// == TO SLAVE ==
// | '*' | '*' |  action (8b)       | length(8b)|         data (N x 8b)       | Check Sum (8b) |
// ---------------------------------------------------------------------------------------------
// | '*' | '*' |  PACK_READ         | N + 1     | N x whatToRead(8b)          | Check Sum (8b) |   <- when this is recived slave will send back N values
// | '*' | '*' |  PACK_WRITE        | N x 5 + 1 | N x (where(8b) + value (32b)| Check Sum (8b) |   <- when this is recived slave will write to its memory N values
// == FROM SLAVE ==
// | '*' | '*' |  PACK_READ_ANSWER  | N x 5 + 1 | N x (what(8b) + value (32b) | Check Sum (8b) |   <- this transfers N values of data

#define PACK_READ					1
#define PACK_WRITE					2
#define PACK_READ_ANSWER			3
//#define PACK_WRITE_ACTUATORS		4
#define PACK_READ_ANSWER_TELE_MAN_DEV	5
#define PACK_READ_ANSWER_TELE_COM_DEV	6

extern char txBuffer[]; 					//wysylana wiadomosc
extern uint16_t txCounter; 					//wysylany aktualnybajt
extern uint16_t txLength; 					//dlugosc wysylanej wiadomosci
extern char rxBuffer[]; 					//odbierana wiadomosc
extern uint16_t rxCounter; 					//aktualnie odbierany bajt
extern char tempSendTab[];

extern uint32_t stateTab[64];				// Array which contains writable / readable slave data.
// 0 - Slave ID
// 1 - ADC_IN11 / Pot 1										// TODO: Rename
// 2 - ADC_IN12 / Pot 2
// 3 - ADC_IN8 / ADC value from strain gauge				// TODO: Calibration
// 4 - ADC_IN13 / ADC value from current sensor in VNH5019. // TODO: Calibration
// 5 - Ofset wzmacniacza
// 6 - Tryb kalibracji (50% mocy)
// 7 - Additional Actions:					//For defines search above.
//			0 = ACTION_NOTHING				// Do nothing additional
//			1 = ACTION_CALIBRATE_FORCE		// Set value: Force shift = Actual Force
//			2 = ACTION_CALIBRATE_POSITION
// 8 - Switch states ( 0 = Off / 1 = On)
#define ST_SWITCH_STATES		stateTab[8]
//			1 lsb = State of switch 1
//			2 lsb = State of switch 2
// 9 - Motor Mode:							//For defines search above.
//			0 = MOTOR_OFF_HIGH_IMP
//			1 = MOTOR_OFF_LOW_IMP
//			2 = MOTOR_DIRECT_CONTROL
//			4 = MOTOR_FORCE_CONTROL
//			8 = MOTOR_SPEED_CONTROL
// 			16 = MOTOR_POSITION_CONTROL
//			+/-
//			256 = MOTOR_POSITION_ONOFF
//			512 = MOTOR_SWITCH_ONOFF
// ************************************ Motor Mode: MOTOR_DIRECT_CONTROL ***
// 10 - Motor PWM (0 - 1000)
#define ST_MOTOR_PWM			stateTab[10]
// 11 - Motor Output config
#define ST_MOTOR_OUTPUT_CONF	stateTab[11]
// 12 - Motor Watchdog
#define ST_MOTOR_WATCHDOG		stateTab[12]
// 13 - BLANK
// ************************************ Motor Mode: MOTOR_FORCE_CONTROL ***
// 14 - Actual force [uNm]				Actual value of force after calibration process.
#define ST_ACTUAL_FORCE			stateTab[14]
//			10 000 000 = 0 uNm
//			 9 999 000 = -1000 uNm
//			10 001 000 = 1000 uNm
// 15 - Target Force [uNm]
#define ST_TARGET_FORCE			stateTab[15]
// 			values: same as above
// 16 - Force shift [uNm]				Used to calibrate force sensor. To generate calibration see 'Additional Actions'.
#define ST_FORCE_SHIFT			stateTab[16]
//			10 000 000 = 0 uNm
//			 9 999 000 = -1000 uNm
//			10 001 000 = 1000 uNm
// 17 - BLANK **************************************************************
// ************************************ Motor Mode: MOTOR_SPEED_CONTROL ***
// 18 - Actual speed [mdeg/sec]
#define ST_ACTUAL_SPEED			stateTab[18]
//			1 000 000 = 0 mdeg/sec
//			  999 000 = -1000 mdeg/sec
//			1 001 000 = 1000 mdeg/sec
// 19 - Target speed [mdeg/sec]
#define ST_TARGET_SPEED			stateTab[19]
// 			values: same as above
// ************************************ Motor Mode: MOTOR_POSITION_CONTROL ***
// 20 - MOtoro actual position [mdeg]
#define ST_ACTUAL_POSITION_IN		stateTab[20]
// 21 - Actuator actual position [mdeg]
#define ST_ACTUAL_POSITION_OUT		stateTab[21]
// 			0 - 359 999 mdeg
// 22 - Target position [mdeg]
#define ST_TARGET_POSITION		stateTab[22]
// 			values: same as above
// 23 - Position shift	[mdeg]			Used to compensate shift from magnetic absolute encoder. ActPos = ( AbsEncoder + PosShift ) mod 360 000
#define ST_POSITION_SHIFT		stateTab[23]
// 24 - Minimum Position [mdeg]			Motor On/Off: Minimum position value that cannot be exceed. If position exceeds motor can rotate in only one direction.
#define ST_MINIMUM_POSITION		stateTab[24]
// 			0 - 359 999 mdeg
// 25 - Maximum Position [mdeg]			Minimum position value that cannot be exceed. If position exceeds motor can rotate in only one direction.
#define ST_MAXIMUM_POSITION		stateTab[25]
// 			0 - 359 999 mdeg
// 26 - BLANK **************************************************************

// 32 - Transmition status
#define ST_TRANS_STAT			stateTab[32]
//			3 lsb - waiting to send PACK_READ_ANSWER_TELE_MAN_DEV
			#define TRANS_READ_ANSWER	4
//			4 lsb - waiting to send answer
			#define TRANS_READ_SEND_ANSWER 8

#endif
