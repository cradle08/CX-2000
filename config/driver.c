// Scope_Eval.c

#ifndef    __DRIVER_C__
#define    __DRIVER_C__

#include "ChainHeader.h"

IO_ UINT8 g_Elec_Status = 0;


//IO_ static UINT32  fac_us=0;							//us��ʱ������			   
//IO_ static UINT32  fac_ms=0;	
//void delay_init()
//{

//	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
//	fac_us=SystemCoreClock/168;				//Ϊϵͳʱ�ӵ�1/8  
//	fac_ms=(u16)fac_us*1000;					//��OS��,����ÿ��ms��Ҫ��systickʱ����   
//}

//void delay_us(u32 nus)
//{		
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
//	SysTick->VAL=0x00;        					//��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
//	SysTick->VAL =0X00;      					 //��ռ�����	 
//}
////��ʱnms
////ע��nms�ķ�Χ
////SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
////nms<=0xffffff*8*1000/SYSCLK
////SYSCLK��λΪHz,nms��λΪms
////��72M������,nms<=1864 
//void delay_ms(u16 nms)
//{	 		  	  
//	u32 temp;		   
//	SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
//	SysTick->VAL =0x00;							//��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
//	SysTick->VAL =0X00;       					//��ռ�����	  	    
//} 


void Delay_US(UINT32 us)
{
    UINT32 start, now, delta, reload, us_tick;
    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while(delta < us_tick * us);
}




void ADC1_DMA_Config()
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream0);
	while(DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){}
	// DMA SET
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&ADC1->DR,
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)g_ADC_Buffer;
	DMA_InitStructure.DMA_DIR					= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize			= ADC_BUFFER_LEN;
	DMA_InitStructure.DMA_PeripheralInc			= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority				= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold			= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	
	//DMA_DoubleBufferModeConfig(DMA2_Stream0,(uint32_t)&g_ADC_Buffer_2,DMA_Memory_0);//DMA_Memory_0?????
	//DMA_DoubleBufferModeCmd(DMA2_Stream0,ENABLE);
	DMA_Init(DMA2_Stream0, &DMA_InitStructure); 
		
	// NVIC
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_HTIF0);
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_HT,ENABLE);	
		
	NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream0_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02;                      
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	//DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
	//while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){}
	DMA_Cmd(DMA2_Stream0, ENABLE);
}

// ADC1 PA5-WBC
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// PA5
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_5;//GPIO_Pin_5, PA5, PA6
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_6;//GPIO_Pin_5, PA5, PA6
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//ADC1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);
	// Common Set
	ADC_CommonInitStructure.ADC_Mode	= ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInit(&ADC_CommonInitStructure);
	// ADC Set
	ADC_InitStructure.ADC_Resolution	= ADC_Resolution_12b;
#if DOUBLE_ADC_CHANNEL
	ADC_InitStructure.ADC_ScanConvMode  = DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
#else
	ADC_InitStructure.ADC_ScanConvMode  = DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
#endif
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv  = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC_DataAlign_Left;//ADC_DataAlign_Right;
	ADC_Init(ADC1, &ADC_InitStructure);	
	ADC_Cmd(ADC1, ENABLE);
	
	ADC1_DMA_Config();
	ADC_DMACmd(ADC1, ENABLE);
	
#if DOUBLE_ADC_CHANNEL
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_3Cycles); //ADC_SampleTime_3Cycles
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_3Cycles); //ADC_SampleTime_3Cycles
#else
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_3Cycles); //ADC_SampleTime_3Cycles
#endif

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	//ADC_SoftwareStartConv(ADC1);
}


//FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG)
//uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx)
void DMA2_Stream0_IRQHandler(void) 
{
	// half DMA_GetFlagStatus
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0) == SET)  
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
		ADC_Status.nSFlag = 1;
		ADC_Status.nID++;
	}
	
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET)  
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
		ADC_Status.nSFlag = 2;
		ADC_Status.nID++;
	}
}


//
void ADC2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	
	// ADC2_CH8, PB0,48V
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// ADC2_CH9, PB1, xiaokong
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// ADC2_CH10, PC0. press
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//ADC2
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, DISABLE);
	// Common Set
	ADC_CommonInitStructure.ADC_Mode	= ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInit(&ADC_CommonInitStructure);
	// ADC Set
	ADC_InitStructure.ADC_Resolution	= ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode  = DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;// ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv  = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC_DataAlign_Left;//ADC_DataAlign_Right;
	ADC_Init(ADC2, &ADC_InitStructure);	
	ADC_Cmd(ADC2, ENABLE);
}

u16 Get_Adc(UINT8 nCh, UINT8 nTime)   
{
	UINT8 i = 0;
	UINT32 nVal = 0;
	
	for(i = 0; i < nTime; i++)
	{
		ADC_RegularChannelConfig(ADC1, nCh, 1, ADC_SampleTime_480Cycles );	    
		ADC_SoftwareStartConv(ADC1);		
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
		nVal += ADC_GetConversionValue(ADC1);	
	}
	nVal /= nTime;
	return nVal;
}

void Reset_Elec_Status(void)
{
	g_Elec_Status = 0;
}

void Set_Elec_Status(void)
{
	g_Elec_Status = 1;
}

UINT8 Get_Elec_Status(void)
{
	return g_Elec_Status;
}

void ELEC_EXIT_FUNC(void)
{
	
	IT_SYS_DlyMs(2);
	if(ELEC_READ == 1)	 
	{
		Set_Elec_Status();
	}		 
	EXTI_ClearITPendingBit(ELEC_EXIT_LINE);
	printf("Elec Exit Func triggle, v=%d, status=%d\r\n", ELEC_READ, Get_Elec_Status());
}

void Elec_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
  
    // 1. enable the input pin Clock 
    RCC_AHB1PeriphClockCmd(ELEC_SRC, ENABLE);
	
    // 2. configure the pin as input floating 
	GPIO_InitStructure.GPIO_Pin = ELEC_PIN;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    GPIO_Init(ELEC_PORT, &GPIO_InitStructure); 

    // 3. extinal-interrupt model 
//    if (eModel == IN_MODEL_EXTI)     
//    {
        // 1). enable the SYSCFG-clock
        RCC_APB2PeriphClockCmd(ELEC_SRC, ENABLE);       
        // 2). Connects EXTI Line to Button GPIO Pin 
        SYSCFG_EXTILineConfig(ELEC_EXIT_SRC, ELEC_EXIT_PIN);    
        // 3). Configure EXTI line 
        EXTI_InitStructure.EXTI_Line = ELEC_EXIT_LINE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  // all pins are rising detected         
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // enable interrupt line 		
        EXTI_Init(&EXTI_InitStructure);                           
        // 4). enable and set input EXTI Interrupt to the lowest priority 
        NVIC_InitStructure.NVIC_IRQChannel = ELEC_EXIT_NUM;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        // enable interrupt
        NVIC_Init(&NVIC_InitStructure);    	
//    }
}

// beep
void Beep_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(BEEP_SRC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
}

void Beep(UINT16 nDelay)
{
	GPIO_SetBits(BEEP_PORT, BEEP_PIN);
	IT_SYS_DlyMs(nDelay);
	GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
}

// pump
void Pump_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(PUMP_CLK_SRC|PUMP_DIR_SRC, ENABLE);
	// clk
	GPIO_InitStructure.GPIO_Pin = PUMP_CLK_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(PUMP_CLK_PORT, &GPIO_InitStructure);
	// dir
	GPIO_InitStructure.GPIO_Pin = PUMP_DIR_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(PUMP_DIR_PORT, &GPIO_InitStructure);
	
	TIM4_PWM_Init(PUMP_PWM_TIM_ARR, PUMP_PWM_TIM_PSC); //84M/84=1M, 1M/500=2k
}

void Pump_Speed_Set(UINT16 nSpeed) // 0-499
{
	// ch2
	if(nSpeed > PUMP_PWM_TIM_ARR) return; 
	TIM_SetCompare2(PUMP_PWM_TIM, nSpeed);
}

void TIM4_PWM_Init(UINT32 Arr, UINT32 Psc)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(PUMP_PWM_TIM_SRC,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(PUMP_CLK_SRC, ENABLE); 	//ʹ��PORTFʱ��	

	//GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); //GPIOF9����Ϊ��ʱ��14

//	GPIO_InitStructure.GPIO_Pin = PUMP_CLK_PIN;           //GPIOF9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
//	GPIO_Init(GPIOF,&GPIO_InitStructure);              //��ʼ��PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=Psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=Arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(PUMP_PWM_TIM,&TIM_TimeBaseStructure);//��ʼ����ʱ��14

	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC1PreloadConfig(PUMP_PWM_TIM, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���

	TIM_ARRPreloadConfig(PUMP_PWM_TIM,ENABLE);//ARPEʹ�� 

	TIM_Cmd(PUMP_PWM_TIM, ENABLE);  //ʹ��TIM14
}
		

void Pump_AntiClockWise(void)
{
	GPIO_SetBits(PUMP_DIR_PORT, PUMP_DIR_PIN);
}

void Pump_ClockWise(void)
{
	GPIO_ResetBits(PUMP_DIR_PORT, PUMP_DIR_PIN);
}

void Pump_Run(UINT16 nUp, UINT16 nDown)
{
	GPIO_SetBits(PUMP_CLK_PORT, PUMP_CLK_PIN);
	Delay_US(nUp);
	GPIO_ResetBits(PUMP_CLK_PORT, PUMP_CLK_PIN);
	Delay_US(nDown);
}

// valve
void Valve_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(VALVE_AIR_SRC|VALVE_LIQUID_SRC, ENABLE);
  // valve air
  GPIO_InitStructure.GPIO_Pin = VALVE_AIR_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(VALVE_AIR_PORT, &GPIO_InitStructure);
  // valve liquid
  GPIO_InitStructure.GPIO_Pin = VALVE_LIQUID_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(VALVE_LIQUID_PORT, &GPIO_InitStructure);
}

void Valve_Air_Exec(UINT8 nOpt)
{
	if(nOpt == EN_OPEN)
	{
		GPIO_SetBits(VALVE_AIR_PORT, VALVE_AIR_PIN);
	}else if(nOpt == EN_CLOSE){
		GPIO_ResetBits(VALVE_AIR_PORT, VALVE_AIR_PIN);
	}
}

void Valve_Liquid_Exec(UINT8 nOpt)
{
	if(nOpt == EN_OPEN)
	{
		GPIO_SetBits(VALVE_LIQUID_PORT, VALVE_LIQUID_PIN);
	}else if(nOpt == EN_CLOSE){
		GPIO_ResetBits(VALVE_LIQUID_PORT, VALVE_LIQUID_PIN);
	}
}

// OC for fix motor, OC for out_in motor
void OC_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(FIX_OC_CLK_SRC|OUT_OC_CLK_SRC|IN_OC_CLK_SRC, ENABLE);
  // fix oc
  GPIO_InitStructure.GPIO_Pin = FIX_OC_CLK_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(FIX_OC_CLK_PORT, &GPIO_InitStructure);
  // oc for out
  GPIO_InitStructure.GPIO_Pin = OUT_OC_CLK_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(OUT_OC_CLK_PORT, &GPIO_InitStructure);
  // oc for in
  GPIO_InitStructure.GPIO_Pin = IN_OC_CLK_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(IN_OC_CLK_PORT, &GPIO_InitStructure);
}

UINT8 Get_Fix_OC_Status(void)
{
	return GPIO_ReadInputDataBit(FIX_OC_CLK_PORT, FIX_OC_CLK_PIN);
}

UINT8 Get_Out_OC_Status(void)
{
	return GPIO_ReadInputDataBit(OUT_OC_CLK_PORT, OUT_OC_CLK_PIN);
}

UINT8 Get_In_OC_Status(void)
{
	return GPIO_ReadInputDataBit(IN_OC_CLK_PORT, IN_OC_CLK_PIN);
}

// fix motor
void Fix_Motor_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(FIX_MOTOR_EN_SRC|FIX_MOTOR_DIR_SRC|FIX_MOTOR_CLK_SRC, ENABLE);
  // en
  GPIO_InitStructure.GPIO_Pin = FIX_MOTOR_EN_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(FIX_MOTOR_EN_PORT, &GPIO_InitStructure);
  // dir
  GPIO_InitStructure.GPIO_Pin = FIX_MOTOR_DIR_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(FIX_MOTOR_DIR_PORT, &GPIO_InitStructure);
  // clk
  GPIO_InitStructure.GPIO_Pin = FIX_MOTOR_CLK_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(FIX_MOTOR_CLK_PORT, &GPIO_InitStructure);
}

void Fix_Motor_Enable(void)
{
	GPIO_SetBits(FIX_MOTOR_EN_PORT, FIX_MOTOR_EN_PIN);
}

void Fix_Motor_Disable(void)
{
	GPIO_ResetBits(FIX_MOTOR_EN_PORT, FIX_MOTOR_EN_PIN);
}

void Fix_Motor_AntiClockWise(void)
{
	GPIO_SetBits(FIX_MOTOR_DIR_PORT, FIX_MOTOR_DIR_PIN);
}

void Fix_Motor_ClockWise(void)
{
	GPIO_ResetBits(FIX_MOTOR_DIR_PORT, FIX_MOTOR_DIR_PIN);
}

void Fix_Motor_Run(UINT16 nUp, UINT16 nDown)
{
	GPIO_SetBits(FIX_MOTOR_CLK_PORT, FIX_MOTOR_CLK_PIN);
	Delay_US(nUp);
	GPIO_ResetBits(FIX_MOTOR_CLK_PORT, FIX_MOTOR_CLK_PIN);
	Delay_US(nDown);
}


// Out_In Motor
void OutIn_Motor_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(OUTIN_MOTOR_EN_SRC|OUTIN_MOTOR_DIR_SRC|OUTIN_MOTOR_CLK_SRC, ENABLE);
  // en
  GPIO_InitStructure.GPIO_Pin = OUTIN_MOTOR_EN_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(OUTIN_MOTOR_EN_PORT, &GPIO_InitStructure);
  // dir
  GPIO_InitStructure.GPIO_Pin = OUTIN_MOTOR_DIR_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(OUTIN_MOTOR_DIR_PORT, &GPIO_InitStructure);
  // clk
  GPIO_InitStructure.GPIO_Pin = OUTIN_MOTOR_CLK_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(OUTIN_MOTOR_CLK_PORT, &GPIO_InitStructure);
}

void OutIn_Motor_Enable(void)
{
	GPIO_SetBits(OUTIN_MOTOR_EN_PORT, OUTIN_MOTOR_EN_PIN);
}

void OutIn_Motor_Disable(void)
{
	GPIO_ResetBits(OUTIN_MOTOR_EN_PORT, OUTIN_MOTOR_EN_PIN);
}

void OutIn_Motor_AntiClockWise(void)
{
	GPIO_SetBits(OUTIN_MOTOR_DIR_PORT, OUTIN_MOTOR_DIR_PIN);
}

void OutIn_Motor_ClockWise(void)
{
	GPIO_ResetBits(OUTIN_MOTOR_DIR_PORT, OUTIN_MOTOR_DIR_PIN);
}

void OutIn_Motor_Run(UINT16 nUp, UINT16 nDown)
{
	GPIO_SetBits(OUTIN_MOTOR_CLK_PORT, OUTIN_MOTOR_CLK_PIN);
	Delay_US(nUp);
	GPIO_ResetBits(OUTIN_MOTOR_CLK_PORT, OUTIN_MOTOR_CLK_PIN);
	Delay_US(nDown);
}

// Digital Register
void SPI3_Init(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;
	  SPI_InitTypeDef  SPI_InitStructure;
		
	  RCC_AHB1PeriphClockCmd(D_REGISTER_CLK_SRC|D_REGISTER_MOSI_SRC|D_REGISTER_CS_SRC, ENABLE);
	  RCC_APB1PeriphClockCmd(D_REGISTER_SPI_SRC, ENABLE); 
		// clk
	  GPIO_InitStructure.GPIO_Pin = D_REGISTER_CLK_PIN; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_Init(D_REGISTER_CLK_PORT, &GPIO_InitStructure);
	  // mosi
	  GPIO_InitStructure.GPIO_Pin = D_REGISTER_MOSI_PIN; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_Init(D_REGISTER_MOSI_PORT, &GPIO_InitStructure);
	  // cs
	  GPIO_InitStructure.GPIO_Pin = D_REGISTER_CS_PIN; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_Init(D_REGISTER_CS_PORT, &GPIO_InitStructure);
	
	  //GPIO_PinAFConfig(D_REGISTER_CLK_PORT,GPIO_PinSource3,GPIO_AF_SPI1); //PB3����Ϊ SPI1
	  //GPIO_PinAFConfig(D_REGISTER_MOSI_PORT,GPIO_PinSource4,GPIO_AF_SPI1); //PB4����Ϊ SPI1
 
	  //����ֻ���SPI�ڳ�ʼ��
	  RCC_APB1PeriphResetCmd(D_REGISTER_SPI_SRC,ENABLE);//��λSPI1
	  RCC_APB1PeriphResetCmd(D_REGISTER_SPI_SRC,DISABLE);//ֹͣ��λSPI1

	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	  SPI_Init(D_REGISTER_SPI, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	 
	  SPI_Cmd(D_REGISTER_SPI, ENABLE); //ʹ��SPI����

	  //SPI1_ReadWriteByte(0xff);//��������		
}



void Driver_Debug(UINT8 nIndex)
{
	UINT16 i = 0;
	switch(nIndex)
	{
		case 0:
		{
			Beep(200);
		}
		break;
		case 1:
		{
			Pump_ClockWise();
			IT_SYS_DlyMs(500);
			Pump_AntiClockWise();
			IT_SYS_DlyMs(500);
			Pump_Speed_Set(100);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			Pump_Speed_Set(400);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
		}
		break;
		case 2: // valve
		{
			Valve_Air_Exec(EN_OPEN);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			Valve_Air_Exec(EN_CLOSE);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			Valve_Liquid_Exec(EN_OPEN);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
			Valve_Liquid_Exec(EN_CLOSE);
			IT_SYS_DlyMs(500);
			IT_SYS_DlyMs(500);
		
		}
		break;
		case 3: //oc
		{
			for(i = 0; i < 10; i++)
			{
				printf("FIX_OC=%d, OUI_OC=%d, IN_OC=%d\r\n", Get_Fix_OC_Status(),\
					Get_Out_OC_Status(), Get_In_OC_Status());
				IT_SYS_DlyMs(500);
				IT_SYS_DlyMs(500);
			}
		
		}
		break;
		case 4:  // fix motor
		{
			Fix_Motor_Enable();
			IT_SYS_DlyMs(500);
			Fix_Motor_Disable();
			IT_SYS_DlyMs(500);
			Fix_Motor_Enable();
			IT_SYS_DlyMs(500);
			Fix_Motor_AntiClockWise();
			IT_SYS_DlyMs(500);
			Fix_Motor_ClockWise();
			IT_SYS_DlyMs(500);
			for(i = 0; i < 2000; i++)
			{
				Fix_Motor_Run(20, 240);
			}
			
		}
		break;
		case 5: // out in motor
		{
			OutIn_Motor_Enable();
			IT_SYS_DlyMs(500);
			OutIn_Motor_Disable();
			IT_SYS_DlyMs(500);
			OutIn_Motor_Enable();
			IT_SYS_DlyMs(500);
			OutIn_Motor_AntiClockWise();
			IT_SYS_DlyMs(500);
			OutIn_Motor_ClockWise();
			IT_SYS_DlyMs(500);
			for(i = 0; i < 2000; i++)
			{
				OutIn_Motor_Run(20, 240);
			}
		}
		break;
		case 6:
		{
		
		}
		break;
		case 7:
		{
		
		}
		break;
		default:break;	
	}	
}














#endif




