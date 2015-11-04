/* Includes */
#include "main.h"
#include <math.h>

int16_t sinTab[360];

int main(void)
{
	Frequency=200;		// UWAGA mHz
	Amplitude=60;	// UWAGA deg
	ZeroPos=180000;		// UWAGA mdeg

	tablicowaniesin();
	InitAll();								// Initialize all peripherals and variables.

	Delay(1000);

	stateTab[9]=MOTOR_POSITION_CONTROL;
	ST_TARGET_POSITION=180000;

	while (1)
	{

//		if( TIM3->CNT >= DELTA_T)
		{
//			TIM3->CNT = 0;
//			StateTabInterpreter();				// Use the appropriate functions depending on the state of stateTab[].
		}

		if(ST_TRANS_STAT == TRANS_READ_ANSWER)
		{
			ST_TRANS_STAT = 0;						// Reset state
			Delay(1);
			GPIO_SetBits(GPIOC, GPIO_Pin_0);		// Direction pin mode: output
			SendDataByUSART(tempSendTab, 4 + tempSendTab[3]);
		}
		else if(ST_TRANS_STAT == TRANS_READ_SEND_ANSWER)
		{
			if( USART_GetFlagStatus(USART3, USART_FLAG_TC) == SET) //oczekiwanie na wys³anie ca³ego pakietu i zmiana bitu kierunku na odczyt
			{
				ST_TRANS_STAT = 0;
				GPIO_ResetBits(GPIOC, GPIO_Pin_0);
			}
		}
	}
}

void tablicowaniesin()
{
	int i;
	double przelicznik=0.01745;
	for(i=0;i<360;i++) sinTab[i] = (int16_t)(1000 * sin((double)i*przelicznik));
};
