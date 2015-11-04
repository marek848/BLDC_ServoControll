/*
 * main_fun.c
 *
 *  Created on: August 12, 2013
 *      Author: Max
 */

#include "main.h"



char txBuffer[256]; 				//wysylana wiadomosc
uint16_t txCounter; 				//wysylany aktualnybajt
uint16_t txLength; 					//dlugosc wysylanej wiadomosci
char rxBuffer[256]; 				//odbierana wiadomosc
uint16_t rxCounter; 				//aktualnie odbierany bajt
char tempSendTab[256];

uint32_t stateTab[64];				// Array which contains writable / readable slave data.

uint8_t menuPosition;

//FuzzyLogic
#define TRESHOLD_FORCE 			30
#define PERCENT_POSITION 		20//maksymalna wielkoœ procentowa regulatora pozycji
#define PERCENT_FORCE 			10 //minimalna wielkosc procentowa regulatora si³y
volatile int32_t OUTPUT_DIR;

// PID FORCE
	volatile int32_t eforce=0;
	volatile int32_t help1=0;
	volatile int32_t help2[2];

struct PID PID_FORCE;

volatile int32_t pidTabForce[4];

volatile int32_t integMemForce;

volatile int32_t derivSumForce;
volatile int32_t derivSumForcePast;
volatile int32_t derivIterForce;
volatile int32_t derivMemForce[DERIV_MEM_SIZE];
// END OF PID FORCE
// PID POSITION
struct PID PID_POSITION;

volatile int32_t pidTabPosition[4];

volatile int32_t integMemPosition;

volatile int32_t derivSumPosition;
volatile int32_t derivSumPositionPast;
volatile int32_t derivIterPosition;
volatile int32_t derivMemPosition[DERIV_MEM_POS_SIZE];
// END OF PID POSITION

uint8_t SendDataByUSART(char *str, uint16_t packetSize)
{
	  //zmienna licznika
	  uint32_t i = 0;
	  //zatrzymanie wystawiania danych do momentu kiedy
	  //poprzednio wystawione dane zostan¹ wys³ane
	  while(txLength != 0){};
	  //skopiowanie ci¹gu znaków do bufora
	  for(i = 0; i < packetSize; i++)
	  {
	    txBuffer[i] = str[i];
	  }
	  //rozpoczêcie wysy³ania danych
	  if(i < 256)
	  {
	    //ustawienie d³ugoœci ci¹gu
	    txLength = i;
	    //wyzerowanie pozosta³ej zawartoœci bufora
	    while(i < 256){txBuffer[i++] = 0x00;};
	    //w³¹czenie przerwania odpowiedzialnego za wysy³anie danych
	    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	    return 1;
	  }
	  //obs³uga b³êdu, gdy wprowadzony ci¹g by³ zbyt d³ugi
	  else
	  {
	    txLength = 0;
	    return 0;
	  }
}

void Delay(uint32_t nTime)
{
	  timingDelay = nTime;
	  while(timingDelay > 0);
}

void TimingDelay_Decrement(void)
{
	if (timingDelay > 0)
	{
		timingDelay--;
	}
}

void WatchDog_Decrement(void)
{
	if (WATCH_DOG > 0)
	{
		WATCH_DOG--;
	}
}

void ActionCalibrateForce(void)
{
	uint32_t tempCalibration = 0;
	int iter;
	for(iter = 0; iter < 10; iter++)
	{
		tempCalibration += ST_ACTUAL_FORCE;
		Delay(2);
	}

	ST_FORCE_SHIFT = tempCalibration/10;
	ADD_ACTION &= ~ACTION_CALIBRATE_FORCE;

}

void ActionCalibratePosition(void)
{

	ST_POSITION_SHIFT = 360000 - ST_ACTUAL_POSITION_OUT;
}

void SwitchReader(void)
{
	if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)ST_SWITCH_STATES |= 1;
	else ST_SWITCH_STATES &= ~(1);

	if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)ST_SWITCH_STATES |= 2;
	else ST_SWITCH_STATES &= ~(2);
}

void ForceConvertion(void)
{
//	ST_ACTUAL_FORCE = 10000000 + ( (stateTab[3] - stateTab[5]) * 708 )  - ( ST_FORCE_SHIFT - 10000000 );
	ST_ACTUAL_FORCE = LOAD_CELL_VALUE ;
}

void PositionConvertion(void)
{
	ST_ACTUAL_POSITION_IN = ( ( ( 360 * TIM1->CCR1 ) / (TIM1->CCR2 / 1000) ) + ST_POSITION_SHIFT ) % 360000;
	ST_ACTUAL_POSITION_OUT = (360000- ( ( 360 * TIM4->CCR1 ) / (TIM4->CCR2 / 1000) ) + ST_POSITION_SHIFT ) % 360000;
}

void PID_SET(void)
{

		PID_FORCE.num[0] = 1;   	// proporcjonalny
		PID_FORCE.den[0] = 3;
		PID_FORCE.num[1] = 0;		// ca³kuj¹cy
		PID_FORCE.den[1] = 1;
		PID_FORCE.num[2] = 1;		// ró¿niczkuj¹cy
		PID_FORCE.den[2] = 10;

//	if (ERROR_VALUE_FORCE < TRESHOLD_FORCE*(-1) || ERROR_VALUE_FORCE > TRESHOLD_FORCE)
//	{	// Kp
//		PID_FORCE.num[0] = 2;
//		PID_FORCE.den[0] = 1;
//		//Ki
//		PID_FORCE.num[1] = 1;
//		PID_FORCE.den[1] = 100;
//		//Kd
//		PID_FORCE.num[2] = 5;
//		PID_FORCE.den[2] = 1000;
//	}
//	else
//	{
//		// Kp
//		PID_FORCE.num[0] = 1;
//		PID_FORCE.den[0] = 1;
//		// Ki
//		PID_FORCE.num[1] = 0;
//		PID_FORCE.den[1] = 1;
//		// Kd
//		PID_FORCE.num[2] = 0;
//		PID_FORCE.den[2] = 1;
//	}
}

void MotorOffHighImp(void)
{
	ST_MOTOR_PWM = 0;
	ST_MOTOR_OUTPUT_CONF = 0;
	ST_MOTOR_WATCHDOG = 1000;

	MotorDirectControl();
}

void MotorOffLowImp(void)
{
	ST_MOTOR_PWM = 0;
	ST_MOTOR_OUTPUT_CONF = 15;
	ST_MOTOR_WATCHDOG = 1000;

	MotorDirectControl();
}

void MotorDirectControl(void)
{
	uint32_t stateTabTemp = 0;																// Using temporary variable to have the same value in each interpreting part

	stateTabTemp = ST_MOTOR_PWM;															// Rounding value to values between 0 - 1000

	if(ST_MOTOR_WATCHDOG != 0)																	//Check Last Time Communication (Watchdog)
	{
		if(stateTabTemp > 999)
		{
			stateTabTemp = 999;
			ST_MOTOR_PWM = stateTabTemp;
		}
		if (stateTab[6]==1) stateTabTemp = ST_MOTOR_PWM/2;



	}
	else
	{
		stateTabTemp = 0;
	}

	TIM2->CCR4 = stateTabTemp;

	if( (ST_MOTOR_OUTPUT_CONF >> 3) % 2 == 1) 	GPIO_WriteBit(GPIOC, GPIO_Pin_4 , Bit_SET);				// Setting ENa
	else 										GPIO_WriteBit(GPIOC, GPIO_Pin_4 , Bit_RESET);

	if( (ST_MOTOR_OUTPUT_CONF >> 2) % 2 == 1) 	GPIO_WriteBit(GPIOC, GPIO_Pin_5 , Bit_SET);				// Setting ENb
	else 										GPIO_WriteBit(GPIOC, GPIO_Pin_5 , Bit_RESET);

 	//TODO: MINMAX POSITION

	if( ((MOTOR_MODE >> 9) % 2 == 1) && (													// Special exception - Motor wants to move to forbidden direction.
			   ( (ST_SWITCH_STATES % 4) == 3) 															// Both switches are ON.
			|| ( (ST_SWITCH_STATES % 4) == (ST_MOTOR_OUTPUT_CONF % 4)) ) ) 								// Motor wants to move in direction of switched ON switch.
	{
		GPIO_WriteBit(GPIOC, GPIO_Pin_6 , Bit_RESET);											// Resetting INa and INb - Low imp to the ground
		GPIO_WriteBit(GPIOC, GPIO_Pin_7 , Bit_RESET);
	}
	else
	{
		if( (ST_MOTOR_OUTPUT_CONF >> 1) % 2 == 1)
											GPIO_WriteBit(GPIOC, GPIO_Pin_6 , Bit_SET);			// Setting INa
		else 								GPIO_WriteBit(GPIOC, GPIO_Pin_6 , Bit_RESET);

		if( (ST_MOTOR_OUTPUT_CONF >> 0) % 2 == 1)
											GPIO_WriteBit(GPIOC, GPIO_Pin_7 , Bit_SET);			// Setting INb
		else 								GPIO_WriteBit(GPIOC, GPIO_Pin_7 , Bit_RESET);
	}



}

void MotorForceCalculation(void)
{
	OUTPUT_P_FORCE = PID_FORCE.num[0] * ERROR_VALUE_FORCE / PID_FORCE.den[0];												// Proportional part, multiply by Kp

	integMemForce += DEL_T_FORCE * ERROR_VALUE_FORCE;																// Integrate part, add to memory
	if( (integMemForce > 0 ) && (integMemForce > INTEG_WINDUP_LIM_FORCE)) integMemForce = INTEG_WINDUP_LIM_FORCE;	// Check windup
	if( (integMemForce < 0 ) && (integMemForce < ((-1) * INTEG_WINDUP_LIM_FORCE))) integMemForce = (-1) * INTEG_WINDUP_LIM_FORCE; // Check windup
	OUTPUT_I_FORCE = PID_FORCE.num[1] * integMemForce / PID_FORCE.den[1];													// Multiply by Ti


	derivMemForce[derivIterForce] = ERROR_VALUE_FORCE;																// Derivative part, add value to memory tab
	derivIterForce = (derivIterForce + 1) % DERIV_MEM_SIZE;															// Modulo counter of memory tab
	uint32_t iter;
	derivSumForce = 0;
	for(iter = 0; iter < DERIV_MEM_SIZE; iter++) derivSumForce += derivMemForce[iter];								// Average deriv from
	OUTPUT_D_FORCE = PID_FORCE.num[2] * (derivSumForce - derivSumForcePast) / (PID_FORCE.den[2] /*DEL_T_FORCE*/);			// Multiply by Td
	derivSumForcePast = derivSumForce;

	OUTPUT_DIR_FORCE = OUTPUT_P_FORCE + OUTPUT_I_FORCE + OUTPUT_D_FORCE;
}
	// Set motor value
void MotorForceControl(void)
{
	PID_SET();

	MotorForceCalculation();
	if(OUTPUT_DIR_FORCE >= 0)
	{
		ST_MOTOR_PWM = (uint32_t)OUTPUT_DIR_FORCE;
		ST_MOTOR_OUTPUT_CONF = 14;
		ST_MOTOR_WATCHDOG = 1000;
	}
	else if(OUTPUT_DIR_FORCE < 0)
	{
		ST_MOTOR_PWM = (uint32_t) ( (-1) * OUTPUT_DIR_FORCE );
		ST_MOTOR_OUTPUT_CONF = 13;
		ST_MOTOR_WATCHDOG = 1000;
	}

	MotorDirectControl();
}

void MotorSpeedControl(void)
{

}

void MotorPositionCalculation(void)
{
	OUTPUT_P_POSITION = PID_POSITION.num[0] * ERROR_VALUE_POSITION / PID_POSITION.den[0];												// Proportional part, multiply by Kp

	integMemPosition += DEL_T_POSITION * ERROR_VALUE_POSITION;																// Integrate part, add to memory
	if( (integMemPosition > 0 ) && (integMemPosition > INTEG_WINDUP_LIM_POSITION)) integMemPosition = INTEG_WINDUP_LIM_POSITION;	// Check windup
	if( (integMemPosition < 0 ) && (integMemPosition < ((-1) * INTEG_WINDUP_LIM_POSITION))) integMemPosition = (-1) * INTEG_WINDUP_LIM_POSITION; // Check windup
	OUTPUT_I_POSITION = PID_POSITION.num[1] * integMemPosition / PID_POSITION.den[1];													// Multiply by Ti


	derivMemPosition[derivIterPosition] = ERROR_VALUE_POSITION;																// Derivative part, add value to memory tab
	derivIterPosition = (derivIterPosition + 1) % DERIV_MEM_POS_SIZE;															// Modulo counter of memory tab
	uint32_t iter;
	derivSumPosition = 0;
	for(iter = 0; iter < DERIV_MEM_POS_SIZE; iter++) derivSumPosition += derivMemPosition[iter];								// Average deriv from
	OUTPUT_D_POSITION = PID_POSITION.num[2] * (derivSumPosition - derivSumPositionPast) / (PID_POSITION.den[2] * DEL_T_POSITION);			// Multiply by Td
	derivSumPositionPast = derivSumPosition;

	OUTPUT_DIR_POSITION = OUTPUT_P_POSITION + OUTPUT_I_POSITION + OUTPUT_D_POSITION;
	OUTPUT_DIR_POSITION;
}
	// Set motor value
void MotorPositionControl(void)
{
	MotorPositionCalculation();
	if(OUTPUT_DIR_POSITION >= 0)
	{
		ST_MOTOR_PWM = (uint32_t)OUTPUT_DIR_POSITION;
		ST_MOTOR_OUTPUT_CONF = 14;
		ST_MOTOR_WATCHDOG = 1000;
	}
	else if(OUTPUT_DIR_POSITION < 0)
	{
		ST_MOTOR_PWM = (uint32_t) ( (-1) * OUTPUT_DIR_POSITION );
		ST_MOTOR_OUTPUT_CONF = 13;
		ST_MOTOR_WATCHDOG = 1000;
	}

	MotorDirectControl();
}

void MotorFuzzyControl()
{
	MotorForceCalculation();

	if(OUTPUT_DIR >= 0)
	{
		ST_MOTOR_PWM = (uint32_t)OUTPUT_DIR;
		ST_MOTOR_OUTPUT_CONF = 14;
		ST_MOTOR_WATCHDOG = 1000;
	}
	else if(OUTPUT_DIR < 0)
	{
		ST_MOTOR_PWM = (uint32_t) ( (-1) * OUTPUT_DIR );
		ST_MOTOR_OUTPUT_CONF = 13;
		ST_MOTOR_WATCHDOG = 1000;
	}
	MotorDirectControl();
}

void StateTabInterpreter(void)
{
	// *** ADC *****************************************************************************

	stateTab[1] = CURRENT_VALUE;
	stateTab[2] = LOAD_CELL_VALUE;
	stateTab[3] = POTENCJOMETER_1_VALUE;

	// *** Switch Reader ******************************************************************

	SwitchReader();

	// *** Force Converter ****************************************************************

	ForceConvertion();			// Converts analog reading to value of momentum.

	// *** Speed Converter ****************************************************************

		// TODO: Function + Encoder program

	// *** Position Converter *************************************************************

	PositionConvertion();		// Converts signal from absolute encoder to value in mdeg.

	// *** Additional Actions   ***********************************************************

	if((ADD_ACTION & ACTION_CALIBRATE_FORCE) == ACTION_CALIBRATE_FORCE) 			ActionCalibrateForce();
	else if((ADD_ACTION & ACTION_CALIBRATE_POSITION) == ACTION_CALIBRATE_POSITION)	 	ActionCalibratePosition();

	// *** Motor Modes Control  ************************************************************

	if	   ((MOTOR_MODE % 256) == MOTOR_OFF_HIGH_IMP)		MotorOffHighImp();
	else if((MOTOR_MODE % 256) == MOTOR_OFF_LOW_IMP)		MotorOffLowImp();
	else if((MOTOR_MODE % 256) == MOTOR_DIRECT_CONTROL)		MotorDirectControl();
	else if((MOTOR_MODE % 256) == MOTOR_FORCE_CONTROL)		MotorForceControl();
	else if((MOTOR_MODE % 256) == MOTOR_SPEED_CONTROL)		MotorSpeedControl(); //TODO
	else if((MOTOR_MODE % 256) == MOTOR_POSITION_CONTROL)	MotorPositionControl(); //TODO
	else if((MOTOR_MODE % 256) == MOTOR_FUZZY_CONTROL) 		MotorFuzzyControl();
}
