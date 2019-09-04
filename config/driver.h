// Scope_Eval.h

#ifndef    __DRIVER_H__
#define    __DRIVER_H__

#include "MyType.h"       // ---
#include <stdio.h>        // for "printf" debug

extern IO_ UINT8 g_Elec_Status;

// elec PE0
#define ELEC_PORT			GPIOE
#define ELEC_PIN			GPIO_Pin_0
#define ELEC_SRC			RCC_AHB1Periph_GPIOB
#define ELEC_EXIT_LINE 		EXTI_Line0
#define ELEC_EXIT_SRC		EXTI_PortSourceGPIOE
#define ELEC_EXIT_PIN		EXTI_PinSource0
#define ELEC_EXIT_NUM		EXTI0_IRQn
#define ELEC_EXIT_FUNC		EXTI0_IRQHandler
#define ELEC_READ			GPIO_ReadInputDataBit(ELEC_PORT, ELEC_PIN)

//beep PF11	
#define BEEP_PORT				GPIOF
#define BEEP_PIN				GPIO_Pin_11
#define BEEP_SRC				RCC_AHB1Periph_GPIOF

// Pump PD13_CLK(TIM4_CH3), PD4_DIR
#define PUMP_CLK_PORT			GPIOD
#define PUMP_CLK_PIN			GPIO_Pin_13
#define PUMP_CLK_SRC			RCC_AHB1Periph_GPIOD

#define PUMP_PWM_TIM			TIM14
#define PUMP_PWM_TIM_SRC		RCC_APB1Periph_TIM14	
#define PUMP_PWM_TIM_ARR		499
#define PUMP_PWM_TIM_PSC		83

#define PUMP_DIR_PORT			GPIOD
#define PUMP_DIR_PIN			GPIO_Pin_4
#define PUMP_DIR_SRC			RCC_AHB1Periph_GPIOD

//switch 1_PG0, 2_PG1
#define VALVE_AIR_PORT			GPIOG
#define VALVE_AIR_PIN			GPIO_Pin_0
#define VALVE_AIR_SRC			RCC_AHB1Periph_GPIOG

#define VALVE_LIQUID_PORT		GPIOG
#define VALVE_LIQUID_PIN		GPIO_Pin_1
#define VALVE_LIQUID_SRC		RCC_AHB1Periph_GPIOG

// fix motor, PD0_EN, PD1_Dir, PD2_CLK
#define FIX_MOTOR_EN_PORT		GPIOD
#define FIX_MOTOR_EN_PIN		GPIO_Pin_0
#define FIX_MOTOR_EN_SRC		RCC_AHB1Periph_GPIOD

#define FIX_MOTOR_DIR_PORT		GPIOD
#define FIX_MOTOR_DIR_PIN		GPIO_Pin_1
#define FIX_MOTOR_DIR_SRC		RCC_AHB1Periph_GPIOD

#define FIX_MOTOR_CLK_PORT		GPIOD
#define FIX_MOTOR_CLK_PIN		GPIO_Pin_2
#define FIX_MOTOR_CLK_SRC		RCC_AHB1Periph_GPIOD

// out_in motor, PD10_EN, PD11_Dir, PD12_CLK
#define OUTIN_MOTOR_EN_PORT		GPIOD
#define OUTIN_MOTOR_EN_PIN		GPIO_Pin_10
#define OUTIN_MOTOR_EN_SRC		RCC_AHB1Periph_GPIOD

#define OUTIN_MOTOR_DIR_PORT	GPIOD
#define OUTIN_MOTOR_DIR_PIN 	GPIO_Pin_11
#define OUTIN_MOTOR_DIR_SRC		RCC_AHB1Periph_GPIOD

#define OUTIN_MOTOR_CLK_PORT	GPIOD
#define OUTIN_MOTOR_CLK_PIN		GPIO_Pin_12
#define OUTIN_MOTOR_CLK_SRC		RCC_AHB1Periph_GPIOD

// OC for fix motor, PE2
#define FIX_OC_CLK_PORT			GPIOE
#define FIX_OC_CLK_PIN			GPIO_Pin_2
#define FIX_OC_CLK_SRC			RCC_AHB1Periph_GPIOE

// OC for out motor, PE3
#define OUT_OC_CLK_PORT			GPIOE
#define OUT_OC_CLK_PIN			GPIO_Pin_3
#define OUT_OC_CLK_SRC			RCC_AHB1Periph_GPIOE

// OC for in motor, PE4
#define IN_OC_CLK_PORT			GPIOE
#define IN_OC_CLK_PIN			GPIO_Pin_4
#define IN_OC_CLK_SRC			RCC_AHB1Periph_GPIOE

// Digital Register(SPI3), PC10_CLK,PC12_MOSI,PC13_CS
#define D_REGISTER_CLK_PORT		GPIOC
#define D_REGISTER_CLK_PIN		GPIO_Pin_10
#define D_REGISTER_CLK_SRC		RCC_AHB1Periph_GPIOC

#define D_REGISTER_MOSI_PORT	GPIOC
#define D_REGISTER_MOSI_PIN		GPIO_Pin_12
#define D_REGISTER_MOSI_SRC		RCC_AHB1Periph_GPIOC

#define D_REGISTER_CS_PORT		GPIOC
#define D_REGISTER_CS_PIN		GPIO_Pin_13
#define D_REGISTER_CS_SRC		RCC_AHB1Periph_GPIOC
#define D_REGISTER_SPI			SPI3
#define D_REGISTER_SPI_SRC 		RCC_APB1Periph_SPI3
//#define D_REGISTER_CLK_SPI_PINSRC 

enum{
	EN_CLOSE	= 0,
	EN_OPEN		= 1
};

void ADC1_Init(void);
void ADC2_Init(void);

void Reset_Elec_Status(void);
void Set_Elec_Status(void);
UINT8 Get_Elec_Status(void);
void Elec_Init(void);

void Beep_Init(void);
void Beep(UINT16 nDelay);

void Pump_init(void);
void TIM4_PWM_Init(UINT32 Arr, UINT32 Psc);
void Pump_Speed_Set(UINT16 nSpeed);
void Pump_AntiClockWise(void);
void Pump_ClockWise(void);
void Pump_Run(UINT16 nUp, UINT16 nDown);

void Valve_Init(void);
void Valve_Air_Exec(UINT8 nOpt);
void Valve_Liquid_Exec(UINT8 nOpt);

void OC_Init(void);
UINT8 Get_Fix_OC_Status(void);
UINT8 Get_Out_OC_Status(void);
UINT8 Get_In_OC_Status(void);

void Fix_Motor_Init(void);
void Fix_Motor_Enable(void);
void Fix_Motor_Disable(void);
void Fix_Motor_AntiClockWise(void);
void Fix_Motor_ClockWise(void);
void Fix_Motor_Run(UINT16 nUp, UINT16 nDown);

void OutIn_Motor_Init(void);
void OutIn_Motor_Enable(void);
void OutIn_Motor_Disable(void);
void OutIn_Motor_AntiClockWise(void);
void OutIn_Motor_ClockWise(void);
void OutIn_Motor_Run(UINT16 nUp, UINT16 nDown);



void Driver_Debug(UINT8 nIndex);






















#endif




