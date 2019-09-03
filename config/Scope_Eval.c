// Scope_Eval.c

#ifndef    __SCOPE_EVAL_C__
#define    __SCOPE_EVAL_C__

#include "ChainHeader.h"
#include "Scope_It.h"

//-----------------------------------------------------------------------------------------
// definition


//-----------------------------------------------------------------------------------------
// the const group

// com-definition
USART_TypeDef* CODE_ COM_USART[COM_NUM]       = {COMM1, COMM2, COMM3}; 
UINT32         CODE_ COM_USART_CLK[COM_NUM]   = {COM1_CLK, COM2_CLK, COM3_CLK};
UINT16         CODE_ COM_ET_IRQn[COM_NUM]     = {COM1_IRQn, COM2_IRQn, COM3_IRQn};
//
GPIO_TypeDef*  CODE_ COM_TX_PORT[COM_NUM]     = {COM1_TX_GPIO_PORT, COM2_TX_GPIO_PORT, COM3_TX_GPIO_PORT};
UINT16         CODE_ COM_TX_PIN[COM_NUM]      = {COM1_TX_PIN, COM2_TX_PIN, COM3_TX_PIN};
UINT32         CODE_ COM_TX_PORT_CLK[COM_NUM] = {COM1_TX_GPIO_CLK, COM2_TX_GPIO_CLK, COM3_TX_GPIO_CLK};
//
GPIO_TypeDef*  CODE_ COM_RX_PORT[COM_NUM]     = {COM1_RX_GPIO_PORT, COM2_RX_GPIO_PORT, COM3_RX_GPIO_PORT};
UINT16         CODE_ COM_RX_PIN[COM_NUM]      = {COM1_RX_PIN, COM2_RX_PIN, COM3_RX_PIN};
UINT32         CODE_ COM_RX_PORT_CLK[COM_NUM] = {COM1_RX_GPIO_CLK, COM2_RX_GPIO_CLK, COM3_RX_GPIO_CLK};
//
UINT8          CODE_ COM_AF_TX_PINSOURCE[COM_NUM] = {COM1_AF_TX_PIN_SOURCE, COM2_AF_TX_PIN_SOURCE, COM3_AF_TX_PIN_SOURCE};
UINT8          CODE_ COM_AF_RX_PINSOURCE[COM_NUM] = {COM1_AF_RX_PIN_SOURCE, COM2_AF_RX_PIN_SOURCE, COM3_AF_RX_PIN_SOURCE};
UINT8          CODE_ COM_AF_UART[COM_NUM] = {COM1_AF_UART, COM2_AF_UART, COM3_AF_UART};

// Output def --- led and speaker
GPIO_TypeDef*  CODE_ OUT_PORT[OUTPUT_NUM]= 
{
    //
    OUT_STATUS_LED1_GPIO_PORT, 
    OUT_STATUS_LED2_GPIO_PORT, 
    OUT_MCU_LED1_GPIO_PORT,
    OUT_MCU_LED2_GPIO_PORT,
    //
    OUT_LAN8720_RST_GPIO_PORT,
    //
    OUT_HEAT1_GPIO_PORT,
	OUT_HEAT1_GPIO_PORT
};
UINT16 CODE_ OUT_PIN[OUTPUT_NUM]= 
{
    //
    OUT_STATUS_LED1_GPIO_PIN, 
    OUT_STATUS_LED2_GPIO_PIN, 
    OUT_MCU_LED1_GPIO_PIN,
    OUT_MCU_LED2_GPIO_PIN,
    //
    OUT_LAN8720_RST_GPIO_PIN,
    //
    OUT_HEAT1_GPIO_PIN,
	OUT_HEAT1_GPIO_PIN
};
UINT32 CODE_ OUT_CLK[OUTPUT_NUM]= 
{
    //
    OUT_STATUS_LED1_GPIO_CLK, 
    OUT_STATUS_LED2_GPIO_CLK, 
    OUT_MCU_LED1_GPIO_CLK,
    OUT_MCU_LED2_GPIO_CLK,
    //
    OUT_LAN8720_RST_GPIO_CLK,
    //
    OUT_HEAT1_GPIO_CLK,
	OUT_HEAT1_GPIO_CLK
};

// Input def  --- commom input and exti interrupt
GPIO_TypeDef* CODE_ IN_PORT[INPUT_NUM]= 
{
    //
    IN_HOME_1_GPIO_PORT,
	IN_HOME_2_GPIO_PORT,
	IN_HOME_3_GPIO_PORT,
	IN_HOME_4_GPIO_PORT,
	IN_HOME_5_GPIO_PORT,
	IN_HOME_6_GPIO_PORT,
	//
	IN_MT_STATUS_GPIO_PORT
};
UINT16 CODE_ IN_PIN[INPUT_NUM]=
{
    //
    IN_HOME_1_GPIO_PIN,
	IN_HOME_2_GPIO_PIN,
	IN_HOME_3_GPIO_PIN,
	IN_HOME_4_GPIO_PIN,
	IN_HOME_5_GPIO_PIN,
	IN_HOME_6_GPIO_PIN,
	//
	IN_MT_STATUS_GPIO_PIN
};	
UINT32 CODE_ IN_CLK[INPUT_NUM]=
{
    //
    IN_HOME_1_GPIO_CLK,
	IN_HOME_2_GPIO_CLK,
	IN_HOME_3_GPIO_CLK,
	IN_HOME_4_GPIO_CLK,
	IN_HOME_5_GPIO_CLK,
	IN_HOME_6_GPIO_CLK,
	//
	IN_MT_STATUS_GPIO_CLK
};
UINT16 CODE_ IN_ET_LINE[INPUT_NUM]=
{
    //
    IN_HOME_1_ET_LINE,
	IN_HOME_2_ET_LINE,
	IN_HOME_3_ET_LINE,
	IN_HOME_4_ET_LINE,
	IN_HOME_5_ET_LINE,
	IN_HOME_6_ET_LINE,
	//
	IN_MT_STATUS_ET_LINE
};
UINT16 CODE_ IN_ET_PORT[INPUT_NUM]=
{
    //
    IN_HOME_1_ET_PORT,
	IN_HOME_2_ET_PORT,
	IN_HOME_3_ET_PORT,
	IN_HOME_4_ET_PORT,
	IN_HOME_5_ET_PORT,
	IN_HOME_6_ET_PORT,
	//
	IN_MT_STATUS_ET_PORT
};
UINT16 CODE_ IN_ET_PIN[INPUT_NUM]=
{
    //
    IN_HOME_1_ET_PIN,
	IN_HOME_2_ET_PIN,
	IN_HOME_3_ET_PIN,
	IN_HOME_4_ET_PIN,
	IN_HOME_5_ET_PIN,
	IN_HOME_6_ET_PIN,
	//
	IN_MT_STATUS_ET_PIN
};
UINT16 CODE_ IN_ET_IRQn[INPUT_NUM]=
{
    //
    IN_HOME_1_ET_IRQn,
	IN_HOME_2_ET_IRQn,
	IN_HOME_3_ET_IRQn,
	IN_HOME_4_ET_IRQn,
	IN_HOME_5_ET_IRQn,
	IN_HOME_6_ET_IRQn,
	//
	IN_MT_STATUS_ET_IRQn
};


//-----------------------------------------------------------------------------------------
// global variables


//-----------------------------------------------------------------------------------------
// define "USE_FULL_ASSERT" in the project options setting
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
	
    /* Infinite loop */
#if 1	
    while (1)
    {
    
    }
#endif

}

//-----------------------------------------------------------------------------------------
// macro defined for "printf" debugging
PUTCHAR_PROTOTYPE
{
	// Place your implementation of fputc here 
	// e.g. write a character to the USART 
#if 0
	
#else

	PL_COM_SendChar(ch);   // will be recovered

#endif

	return ch;
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
		nVal =+ ADC_GetConversionValue(ADC1);	
	}
	nVal /= nTime;
	return nVal;
}


//-----------------------------------------------------------------------------------------
// initialization for the code block
void EVAL_Init(void)
{
    USART_InitTypeDef USART_InitStructure;	

	//-------------------------------------------
    // 1. update the system's clock
    SystemCoreClockUpdate();
	printf("\r\n--- SystemCoreClock = %d ---\r\n", SystemCoreClock);

    //-------------------------------------------
    // 2. Initialize and start the SysTick counter and its interrupt. 
    //    take attention: when want to use IRQ_DelayMs(), this must be called first !!!
	if(SysTick_Config(SystemCoreClock / 1000))   // Setup SysTick Timer for 1 millisecond interrupts. 
    { 
        // Capture error / 
	    printf("\r\nSysTick_Config error\r\n");
        while (1);
    }
    // attentio: set to the higher priority
	NVIC_SetPriority(SysTick_IRQn, 0);
    // IT_SYS_DlyMs(100);
    
    //-------------------------------------------
	// 3. priority setting
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  
	
    //-------------------------------------------   
	// Disable the JTAG interface and enable the SWJ interface. // GPIO_Remap_SWJ_JTAGDisable	
	// Enable the AFIO Clock  
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	// mapping
    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);     // AFIO->MAPR |= 0x02000000; // GPIO_Remap_SWJ_JTAGDisable

    /*******************************************/
    // 4. initialize  com 
	USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // 
    EVAL_ComInit(E_COM_MAIN, &USART_InitStructure);  
////   EVAL_ComInit(E_COM_SAMPLING, &USART_InitStructure);  
	// EVAL_ComInit(E_COM_NULL, &USART_InitStructure);  

	//-------------------------------------------
	// 5. Initialize  IO output  
	// led and the reset pin of the FPGA 
////    EVAL_OutputInit(O_STATUS_LED_1);
////	EVAL_OutputInit(O_STATUS_LED_2);
	EVAL_OutputInit(O_MCU_LED_1);
	EVAL_OutputInit(O_MCU_LED_2);
	EVAL_OutputInit(O_LAN8720_RST);
////	EVAL_OutputInit(O_HEAT_1);
////	EVAL_OutputInit(O_HEAT_2);
	
	//-------------------------------------------
    // 6. FPGA init
    IT_SYS_DlyMs(500); // attention: waiting the FPGA to be ready
#if !USE_STM32F407_ONLY
	FPGA_Init();
	FPGA_ResetHardware();
#endif

	//-------------------------------------------
	// 7. initialize  IO input --- normal and exti interrupt 
	// input
////	EVAL_InputInit(I_HOME_X, IN_MODEL_GPIO);      // 
////	EVAL_InputInit(I_HOME_Y, IN_MODEL_GPIO);      // 
////	EVAL_InputInit(I_HOME_Z, IN_MODEL_GPIO);      // 
////	EVAL_InputInit(I_HOME_M, IN_MODEL_GPIO);      // 
#if !USE_STM32F407_ONLY
	EVAL_InputInit(I_FEEDBACK_1, IN_MODEL_EXTI);  //EXTI15-10 PB13// EXTI9_5_IRQn  EXTI_Line7
#endif
	//-------------------------------------------
	// 8. Initialize the spi-flash 
    // SPI_FLASH_Init();
    
    //-------------------------------------------
	// 9. the timer of the system messages
	PF_InitTimer2();
	
	
#if USE_STM32F407_ONLY
	ADC1_Init();
#endif
}


//-----------------------------------------------------------------------------------------
// com control
void EVAL_ComInit(Com_TypeDef eCom, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;

    // 1. enable GPIO clock 
	RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[eCom] | COM_RX_PORT_CLK[eCom], ENABLE);
    // 2. enable UART clock 
    if (eCom == E_COM_SAMPLING)   // com_2  
    {
        RCC_APB1PeriphClockCmd(COM_USART_CLK[eCom], ENABLE);  // APB1
    }
    else                 // com_1 or com_3
    {
        RCC_APB2PeriphClockCmd(COM_USART_CLK[eCom], ENABLE);  // APB2
    }
	// 3. configure AFIO  
	GPIO_PinAFConfig(COM_TX_PORT[eCom],COM_AF_TX_PINSOURCE[eCom],COM_AF_UART[eCom]); // tx-pin
	GPIO_PinAFConfig(COM_RX_PORT[eCom],COM_AF_RX_PINSOURCE[eCom],COM_AF_UART[eCom]); // rx-pin
    // 4. IO configuration 
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[eCom] | COM_RX_PIN[eCom]; // tx-pin and rx-pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // AF model
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 50MHz 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // push output
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 
    GPIO_Init(COM_TX_PORT[eCom],&GPIO_InitStructure);
	// 5. USART configuration 
    USART_Init(COM_USART[eCom], USART_InitStruct);	
    // 6. Enable USART
    USART_Cmd(COM_USART[eCom], ENABLE);
	// 7. enable Receive and Transmit interrupts 
    USART_ITConfig(COM_USART[eCom], USART_IT_TC, ENABLE);      //  TX_INT = 1;
    USART_ITConfig(COM_USART[eCom], USART_IT_RXNE, ENABLE);    //  RX_INT = 1;	
	// 8. configure and enable USART interrupt  
    NVIC_InitStructure.NVIC_IRQChannel = COM_ET_IRQn[eCom];       
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}

//-----------------------------------------------------------------------------------------
// GPIO output contorl 
void EVAL_OutputInit(Output_TypeDef eOut)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    // 1. enable the GPIO Clock  
    RCC_AHB1PeriphClockCmd(OUT_CLK[eOut], ENABLE);  // AHB1

    // 2. configure the GPIO pin
	GPIO_InitStructure.GPIO_Pin = OUT_PIN[eOut]; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_Init(OUT_PORT[eOut], &GPIO_InitStructure);  // init gpio
}

void EVAL_OutputSet(Output_TypeDef eOut)
{
    OUT_PORT[eOut]->BSRRL = OUT_PIN[eOut];
}

void EVAL_OutputClr(Output_TypeDef eOut)
{
    OUT_PORT[eOut]->BSRRH = OUT_PIN[eOut];
}

void EVAL_OutputToggle(Output_TypeDef eOut)
{
    OUT_PORT[eOut]->ODR ^= OUT_PIN[eOut];
}

//-----------------------------------------------------------------------------------------
// GPIO input contorl 
void EVAL_InputInit(Input_TypeDef eIn, InModel_Typedef eModel)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
  
    // 1. enable the input pin Clock 
    RCC_AHB1PeriphClockCmd(IN_CLK[eIn], ENABLE);
  
    // 2. configure the pin as input floating 
	GPIO_InitStructure.GPIO_Pin = IN_PIN[eIn];  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_Init(IN_PORT[eIn], &GPIO_InitStructure); 

    // 3. extinal-interrupt model 
    if (eModel == IN_MODEL_EXTI)     
    {
        // 1). enable the SYSCFG-clock
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);       
        // 2). Connects EXTI Line to Button GPIO Pin 
        SYSCFG_EXTILineConfig(IN_ET_PORT[eIn], IN_ET_PIN[eIn]);    
        // 3). Configure EXTI line 
        EXTI_InitStructure.EXTI_Line = IN_ET_LINE[eIn];
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  // all pins are rising detected         
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // enable interrupt line 		
        EXTI_Init(&EXTI_InitStructure);                           
        // 4). enable and set input EXTI Interrupt to the lowest priority 
        NVIC_InitStructure.NVIC_IRQChannel = IN_ET_IRQn[eIn];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        // enable interrupt
        NVIC_Init(&NVIC_InitStructure);    	
    }


}

UINT8 EVAL_InputGetState(Input_TypeDef eIn)
{
    return GPIO_ReadInputDataBit(IN_PORT[eIn], IN_PIN[eIn]);
}


//-----------------------------------------------------------------------------------------
// 
UINT8  PF_InitTimer2(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 

    // reset the timerX
	TIM_DeInit(TIM2); 
    // enable the clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	// interrupt configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
	//----------------------------------------------------------------
	// TIMx configuration  11059200 * 15 = (256*432*100) * 15 = 165888000  
	// attention: "== SYSCLK / 3"
	//            ".TIM_Period" is autoreload value, and must bigger 
	//            than 1000, or it case many interrupts too frequently
	//            that consumes all the interrupt-resource of the MCU
    TIM_TimeBaseStructure.TIM_Period = 6480;                      // 10ms, 100Hz = 648000 / 6480
    TIM_TimeBaseStructure.TIM_Prescaler = 256;                    // 165888000 / 256 = 648000 Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;       // none   
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // up 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
    //----------------------------------------------
	// enable the autoreload 
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	// Clear TIMx update pending flag
    TIM_ClearFlag(TIM2, TIM_FLAG_Update); 
    // Enable TIMx Update interrupt 
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	// TIMx disable counter
    TIM_Cmd(TIM2, ENABLE);   

    
	return e_Feedback_Success;
}


#endif


