/*
 * init_fun.c
 *
 *  Created on: August 12, 2013
 *      Author: Max
 */

#include "main.h"

void InitRCC(void)
{
	RCC_DeInit();											// Reset clock.
	RCC_HSEConfig(RCC_HSE_ON);								// HSE on - 8MHz crystal oscillator.
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) {}	// Waiting for HSE ready state.
	FLASH_SetLatency(FLASH_Latency_2);						// Stitch-on delay for FLASH.
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);						// HCL = SYSCLK. Set AHB frequency to be equal to RCC frequency.
	RCC_PCLK2Config(RCC_HCLK_Div1);							// PCLK2 = HCLK. Set APB2 frequency to be equal to RCC frequency.
	RCC_PCLK1Config(RCC_HCLK_Div2);							// PCLK1 = HCLK/2. Maximum possible frequency on this rail is 36MHz.
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	// PLLCLK = 8MHz * 9 = 72 MHz. HSE frequency multiplication.
	RCC_PLLCmd(ENABLE);										// PLL switch-on.
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);	// Waiting for PLL ready.
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);				// Set PLL as source for system clock.
	while (RCC_GetSYSCLKSource() != 0x08) {					// Waiting for system clock ready.
	}
}

void GPIO_Conf_AllToAIN(void)
{
	/* Initialize all pins as input with high impedance. */
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // Port A
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			// Mode = Analog Input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // Port B
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			// Mode = Analog Input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void InitNVIC(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitTypeDef NVIC_InitStructure;

//    /* Enable interrupt from switches. */
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

	/* Enable interrupt from USART1 */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable interrupt from DMA1 */
	NVIC_InitTypeDef  NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/* Enable interrupt from TIM3 */
	NVIC_InitTypeDef  NVIC_InitStruc;
	NVIC_InitStruc.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStruc.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStruc.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruc);

}

void InitSwitch(void)
{
	/* Initialize input pins that will be connected to switches. */
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // Port A

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void InitADC(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6) ;													//72Mhz/6=12MHz

	ADC_InitTypeDef  ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right ;
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None ;
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel=ADC_TAB_SIZE;													// Number of channels
	ADC_InitStruct.ADC_ScanConvMode=ENABLE;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_CURRENT, 1, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_LOAD_CELL, 2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_POTENCJOMETER_2, 3, ADC_SampleTime_71Cycles5);

	ADC_Cmd(ADC1, ENABLE);																// Enabling ADC.

	ADC_ResetCalibration( ADC1 );														// Calibration.
	while(ADC_GetResetCalibrationStatus( ADC1 ));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus (ADC1) == SET);

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	ADC_DMACmd(ADC1, ENABLE);
}

void InitDMA(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStruct;

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_BufferSize=sizeof(adcTab)/sizeof(uint16_t);				// DMA buffer size ( in bytes).
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;								// Type of source of DMA transmition.
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;										// M2M disabled.
	DMA_InitStruct.DMA_MemoryBaseAddr=(uint32_t)&(adcTab);						// Base address.
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;				// Memory size - half word.
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;							// Used when array is bigger than 1 cell.
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;									// This enable DMA to work continuously.
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(ADC1->DR);				// Source address.
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;		// Peripheral size - half word (ADC DR is 16-bit).
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;					// Peripheral data address is always the same.
	DMA_InitStruct.DMA_Priority=DMA_Priority_Medium;							// Set DMA priority.

	DMA_Init(DMA1_Channel1, &DMA_InitStruct);									// Initialize channel.
	DMA_Cmd(DMA1_Channel1, ENABLE);												// Enable DMA channell.

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);								// Configure interrupts.
}

void InitUSART(void)
{
	  //w³¹czenie zegara portu
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	  //struktura inicjalizacyjna portu
	  GPIO_InitTypeDef  GPIO_InitStructure;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_WriteBit(GPIOC, GPIO_Pin_0 , Bit_RESET);				// Ustawienie pinu kierunku

		//w³¹czenie zegara USARTa
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	  //Pin PB10 - linia nadawcza USART3
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;                    //piny które chcemy skonfigurowaæ
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;               //tryb wyjœæ portu - wyjœcie push-pull
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;             //szybkoœæ wyjœcia
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  //Pin PB11 - linia odbiorcza USART3-
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;                   //piny które chcemy skonfigurowaæ
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;         		//tryb wejscia
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;             //szybkoœæ wejœcia
	  GPIO_Init(GPIOB, &GPIO_InitStructure);



	  USART_InitTypeDef USART_InitStructure;
	  USART_InitStructure.USART_BaudRate=115200;//19200;                     //Prêdkoæ transmisji
	  USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx; //Tryb pracy - w³¹czony odbiornik oraz nadajnik
	  USART_InitStructure.USART_Parity=USART_Parity_No;             //Bity parzystosci - brak
	  USART_InitStructure.USART_StopBits=USART_StopBits_1;          //Bity stopu - 1
	  USART_InitStructure.USART_WordLength=USART_WordLength_8b;     //D³ugosc ramki - 8 bitów
	  USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //Sprzêtowa kontrola przep³ywu - brak
	  USART_Init(USART3,&USART_InitStructure);

	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	  //Uruchomienie USART3
	  USART_Cmd(USART3,ENABLE);
}

void InitPWMMotorInterface(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Direction and enable pins initialization */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOC, GPIO_Pin_4 , Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_5 , Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_6 , Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_7 , Bit_RESET);

	/* 1 x PWM Initialization */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

//////Inicjalizowanie wyjsc PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//////w³¹czenie zegara dla TIM

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef  TIM_OCInitStruct;

//////konfiguracja TIM w tryb do sterowania serwami
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period= 1000 - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler= 720 - 1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);

//////konfiguracja wyjcia PWM
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=500;									//100-200

	TIM_OC4Init(TIM2, &TIM_OCInitStruct);

	TIM_OC4PreloadConfig(TIM2,ENABLE);

	TIM_ARRPreloadConfig(TIM2,ENABLE);

//////uruchomienie wyjcia (w przypadku TIM1 pamiêtaæ o dodatkowej funkcji)
	TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);

	TIM_Cmd(TIM2,ENABLE);
}

void InitPulseCounter(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

//////w³¹czenie zegara dla TIM

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef  TIM_OCInitStruct;

//////konfiguracja TIM w tryb do sterowania serwami
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period= 65536 - 1;	// Maximal
	TIM_TimeBaseInitStruct.TIM_Prescaler= 72 - 1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);

////////konfiguracja wyjcia PWM
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=DELTA_T;

	TIM_OC1Init(TIM3, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(TIM3,DISABLE);

	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);

	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);


	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=65535;	//Maximal

	TIM_OC2Init(TIM3, &TIM_OCInitStruct);

	TIM_OC2PreloadConfig(TIM3,DISABLE);

	TIM_ARRPreloadConfig(TIM3,ENABLE);

	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);

	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);

	TIM_Cmd(TIM3,ENABLE);
}

void InitAbsoluteEncoderInterface(void)
{
	/* TIM3 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOA clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 channel 2 pin (PA.09) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_ICInitTypeDef  TIM_ICInitStructure;

	TIM1->PSC = 1;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;


	TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

	/* Select the TIM2 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */
	TIM_Cmd(TIM1, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/* TIM2 channel 2 pin (PA.01) configuration */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM4->PSC = 1;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

	/* Select the TIM2 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);

//		/*************************************************************************/
//		TIM4->CCMR1 |= 1;
//		TIM4->CCER &= ~( 1 );
//		TIM4->CCMR1 |= 1 << 8;
//		TIM4->CCER |=1 << 5;
//		TIM4->SMCR |= 5 << 4;
//		TIM4->SMCR |= 4;
//		TIM4->CCER |= 1;
//		TIM4->CCER |= 1 << 4;
//		TIM_Cmd(TIM4, ENABLE);
		/*************************************************************************/
}

void InitIncrementalEncoderInterface()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 65535; // Maximal
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM4, ENABLE);
}

void InitStateTab(void)
{
	int j;
	for( j = 0; j < 64; j++)
	{
		stateTab[j] = 0;
	}

	stateTab[0] = DEVICE_ID;											// Id
	ST_TARGET_FORCE = 10000000;									// Base target force value ( 0 Nm )
	ST_FORCE_SHIFT = 10000000;									// 0 uNm shift.
	stateTab[5]=2047; 												// ofset wzmacniacza
}

void InitAll(void)
{
	/* Variables */
	int i;														// Variable used in 'for' loops.

//	for( i = 0; i < ADC_AVERAGE_COUNT; i++) adcTemp[i]=0;
	adcCount = 0;												// Used by DMA interrupt to average ADC value.
	for( i = 0; i <  ADC_TAB_SIZE ; i++)adcTabPart[i] = 0;		// Used by DMA interrupt to average ADC value.

	rxCounter = 0;
	txCounter = 0;												// Used to know which byte is sending.
	txLength = 0;												// Used to know how much bytes we have to send.

	timingDelay = 0;

	PID_FORCE.num[0] = 4;   	// proporcjonalny
	PID_FORCE.den[0] = 6;
	PID_FORCE.num[1] = 0;		// ca³kuj¹cy
	PID_FORCE.den[1] = 65000;
	PID_FORCE.num[2] = 0;		// ró¿niczkuj¹cy
	PID_FORCE.den[2] = 5;

	PID_POSITION.num[0] = 1;   	// proporcjonalny
	PID_POSITION.den[0] = 300;
	PID_POSITION.num[1] = 1;		// ca³kuj¹cy
	PID_POSITION.den[1] = 22500;
	PID_POSITION.num[2] = 1;		// ró¿niczkuj¹cy
	PID_POSITION.den[2] = 50;

	integMemForce = 0;
	derivSumForce = 0;
	derivIterForce = 0;
	for(i = 0; i < DERIV_MEM_SIZE; i++) derivMemForce[i]=0;
	for(i = 0; i < 4; i++) pidTabForce[i] = 0;

	integMemPosition = 0;
	derivSumPosition = 0;
	derivIterPosition = 0;
	for(i = 0; i < DERIV_MEM_POS_SIZE; i++) derivMemPosition[i]=0;
	for(i = 0; i < 4; i++) pidTabPosition[i] = 0;

	InitStateTab();												// Asigning values to stateTab[] that will be available before serial communication with master device.

	/* Pripherals inside microprocessor */
	InitRCC();													// Initialize system clocks.
	GPIO_Conf_AllToAIN();										// Initialize all pins as input with high impedance.
	InitNVIC();													// Enable interrupts.
	InitSwitch();
	InitADC();
	InitDMA();
	InitUSART();												//
	InitPWMMotorInterface();
	InitPulseCounter();
	InitAbsoluteEncoderInterface();
//	InitIncrementalEncoderInterface();

	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while (1); 												// In case of an error the program hangs in an infinite loop.
	}

}
