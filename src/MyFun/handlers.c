/*
 * handlers.c
 *
 *  Created on: August 12, 2013
 *      Author: Max
 */

#include "main.h"


//static uint16_t adcTemp[ADC_AVERAGE_COUNT];
int64_t tmpSG[ADC_TAB_SIZE];
int32_t sgwag[21]={-171,-76,9,84,149,204,249,284,309,324,329,324,309,284,249,204,149,84,9,-76,-171};


void DMA1_Channel1_IRQHandler(void)
{
	// This interrupt is released after DMA completes reading from all ADC channels. adcTab[] contains new data.

		// It is used to sum N number of probes from one channel and divide it by N. To complete this cycle
		// this interrupt must be executed N times. Each time after one full cycle of reading all ADC channels.

		DMA_ClearFlag (DMA1_FLAG_TC1);
		/*filtr wed³ug sg*/
		if(adcCount < ADC_AVERAGE_COUNT)
		{
			for (int i=0;i<ADC_TAB_SIZE;i++) tmpSG[i] += adcTab[i] * sgwag[adcCount];
			adcCount++;
		}
		else
		{
			for (int i=0;i<ADC_TAB_SIZE;i++)
			{
				adcTabAverage[i] = (tmpSG[i] + (adcTab[i] * sgwag[adcCount]) ) / 3059;
				tmpSG[i] = 0;
			}
			adcCount = 0;
		}
}

void USART3_IRQHandler(void)
{
	// USART reading part.
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		if( ( rxCounter == 0 ))
		{
			// Check first byte.
			if(USART_ReceiveData(USART3) == '*')
			{
				rxBuffer[rxCounter] = '*';
				rxCounter++;
			}
			else rxCounter = 0;
		}
		else if( ( rxCounter == 1 ) )
		{
			rxBuffer[rxCounter] = USART_ReceiveData(USART3);
			if( rxBuffer[rxCounter] == DEVICE_ID)
			{
				rxCounter++;
			}
			else if(rxBuffer[rxCounter] == '*')
			{
				rxCounter = 1;
			}
			else
			{
				rxCounter = 0;
			}
		}
		else if( rxCounter == 2 )
		{
			// Read "action" byte.
			rxBuffer[rxCounter] = USART_ReceiveData(USART3);

			// Check if value is supported.
			if( rxBuffer[rxCounter] == PACK_READ 		||
				rxBuffer[rxCounter] == PACK_WRITE 			) rxCounter++;
			else rxCounter = 0;

		}
		else if( rxCounter == 3 )
		{
			// Read "length" byte.
			rxBuffer[rxCounter] = USART_ReceiveData(USART3);

			rxCounter++;

		}
		else if( ( rxCounter > 3 ) && ( rxCounter < ( 3 + rxBuffer[3] ) ) )
		{
			// Read N "data" bytes.
			rxBuffer[rxCounter] = USART_ReceiveData(USART3);

			rxCounter++;
		}
		else if( rxCounter == ( 3 + rxBuffer[3]) )
		{
			uint8_t tempCS = 0;
			uint32_t tempSumCS = 0;
			int i;

			// Read "Check Sum" byte.
			rxBuffer[rxCounter] = USART_ReceiveData(USART3);

			// Compute "Check Sum".
			for( i = 0; i < rxBuffer[3] + 1; i++) tempSumCS += rxBuffer[i + 2];
			tempCS = ~( tempSumCS % 256 );

			// Check "Check Sum".
			if( tempCS == (uint8_t)rxBuffer[3 + rxBuffer[3] ])
			{
				switch( rxBuffer[2] )
				{
					case PACK_READ:

						tempSendTab[0] = '*';
						tempSendTab[1] = 0;		// Master ID == 0
						#ifdef	MANIPULATOR_DEVICE
						tempSendTab[2] = PACK_READ_ANSWER_TELE_MAN_DEV;
						#else
						tempSendTab[2] = PACK_READ_ANSWER_TELE_COM_DEV;
						#endif
						tempSendTab[3] = ( rxBuffer[3] - 1 ) * 5 + 2;		// Length
						tempSendTab[4] = stateTab[0] % 256;

						for( i = 0; i < rxBuffer[3] - 1; i++)
						{
							tempSendTab[5 + 5 * i] = rxBuffer[4 + i];
							tempSendTab[6 + 5 * i] = ( stateTab[ (uint32_t)(rxBuffer[4 + i]) ] >> 24 ) % 256;
							tempSendTab[7 + 5 * i] = ( stateTab[ (uint32_t)(rxBuffer[4 + i]) ] >> 16 ) % 256;
							tempSendTab[8 + 5 * i] = ( stateTab[ (uint32_t)(rxBuffer[4 + i]) ] >> 8  ) % 256;
							tempSendTab[9 + 5 * i] = ( stateTab[ (uint32_t)(rxBuffer[4 + i]) ]       ) % 256;
						}

						/* Check Sum */
						tempSendTab[ 3 + tempSendTab[3] ] = 0; 		// Clear "Check Sum" before computing and sending.
						tempSumCS = 0;

						for( i = 0; i < tempSendTab[3] + 1; i++)
						{
							tempSumCS += tempSendTab[ i + 2];
						}

						tempSendTab[ 3 + tempSendTab[3] ] = ~ ( tempSumCS % 256 );

						/* Send Packet */
						// Sending Moved to main.c
						ST_TRANS_STAT = TRANS_READ_ANSWER;


						break;

					case PACK_WRITE:

						for( i = 0; i < ( rxBuffer[3] - 1) / 5; i++)
						{
							stateTab[ (uint8_t)(rxBuffer[ 4 + 5 * i]) ] =
									( ( rxBuffer[ 5 + 5 * i] & 0xFF ) << 24 ) +
									( ( rxBuffer[ 6 + 5 * i] & 0xFF ) << 16 ) +
									( ( rxBuffer[ 7 + 5 * i] & 0xFF ) << 8  ) +
									  ( rxBuffer[ 8 + 5 * i] & 0xFF );

						}
						break;
				}
			}

			rxCounter = 0;
		}
		else rxCounter = 0;
	}

	// USART transmitting part.
	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		// Send byte after byte.
		if(txCounter < txLength)
		{
			USART_SendData(USART3, txBuffer[txCounter]);
			txCounter++;
		}
		else
		{
			// If all bytes are send clear interrupt from TXE.
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

			// Reset transmitter counter.
			txCounter = 0;
			txLength = 0;

			ST_TRANS_STAT = TRANS_READ_SEND_ANSWER;
		}
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3,TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);

		StateTabInterpreter();				// Use the appropriate functions depending on the state of stateTab[].

		TIM_SetCompare1(TIM3, TIM_GetCapture2(TIM3)+DELTA_T);
	}


	if (TIM_GetITStatus(TIM3,TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);

		static uint16_t indeks=0;
		static uint8_t j=0;
		uint16_t t;
		uint32_t tmp;
		uint16_t repetition;

		tmp=2777777/Frequency;
		if( tmp <= 65535 )
		{
			t=(uint16_t)tmp;
			ST_TARGET_POSITION = (sinTab[indeks % 360] * Amplitude) + ZeroPos;
			indeks++;

			TIM_SetCompare2(TIM3, TIM_GetCapture2(TIM3)+t);
		}
		else
		{
			repetition = tmp/65535;
			if ( j == repetition - 1 ) TIM_SetCompare2(TIM3, tmp - (repetition * 65535) );
			else
			{
				TIM_SetCompare2(TIM3, 65535 );
				if ( j > repetition - 1 )
				{
					ST_TARGET_POSITION = (sinTab[indeks % 360] * Amplitude) + ZeroPos;
					indeks++;
					j=0;
				}
			}
			j++;
		}
		if ((indeks / 360 >= 3) && (stateTab[0] != 1))
		{
			Frequency+=200;
			if (Frequency == 1000) Frequency = 200;
			indeks = 0;
		}
	}
}
