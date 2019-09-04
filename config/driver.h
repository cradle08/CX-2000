// Scope_Eval.h

#ifndef    __DRIVER_H__
#define    __DRIVER_H__

#include "MyType.h"       // ---
#include <stdio.h>        // for "printf" debug


void ADC1_Init(void);
void ADC2_Init(void);

// elec PE0
#define ELEC_PORT	GPIOE
#define ELEC_PIN	GPIO_PIN_0
#define ELEC_SRC	RCC_AHB1Periph_GPIOB

//beep PF11
#define BEEP_PORT	GPIOF
#define BEEP_PIN	GPIO_PIN_11
#define BEEP_SRC	RCC_AHB1Periph_GPIOF

// beng PD13_PWM(TIM4_CH3), PD4_DIR
#define BENG_PWM_PORT	GPIOD
#define BENG_PWM_PIN	GPIO_PIN_13
#define BENG_PWM_SRC	RCC_AHB1Periph_GPIOD

#define BENG_DIR_PORT	GPIOD
#define BENG_DIR_PIN	GPIO_PIN_4
#define BENG_DIR_SRC	RCC_AHB1Periph_GPIOD

//switch 1_PG0, 2_PG1
#define SWITH1_PORT	GPIOG
#define SWITH1_PIN	GPIO_PIN_0
#define SWITH1_SRC	RCC_AHB1Periph_GPIOG

#define SWITH2_PORT	GPIOG
#define SWITH2_PIN	GPIO_PIN_1
#define SWITH2_SRC	RCC_AHB1Periph_GPIOG

// fix motor, PD0_EN, PD1_Dir, PD2_CLK
#define FIX_MOTOR_EN_PORT	GPIOD
#define FIX_MOTOR_EN_PIN	GPIO_PIN_0
#define FIX_MOTOR_EN_SRC	RCC_AHB1Periph_GPIOD

#define FIX_MOTOR_DIR_PORT	GPIOD
#define FIX_MOTOR_DIR_PIN	GPIO_PIN_1
#define FIX_MOTOR_DIR_SRC	RCC_AHB1Periph_GPIOD

#define FIX_MOTOR_CLK_PORT	GPIOD
#define FIX_MOTOR_CLK_PIN	GPIO_PIN_2
#define FIX_MOTOR_CLK_SRC	RCC_AHB1Periph_GPIOD

// out_in motor, PD10_EN, PD11_Dir, PD12_CLK
#define OUTIN_MOTOR_EN_PORT	GPIOD
#define OUTIN_MOTOR_EN_PIN	GPIO_PIN_10
#define OUTIN_MOTOR_EN_SRC	RCC_AHB1Periph_GPIOD

#define OUTIN_MOTOR_DIR_PORT	GPIOD
#define OUTIN_MOTOR_DIR_PIN	GPIO_PIN_11
#define OUTIN_MOTOR_DIR_SRC	RCC_AHB1Periph_GPIOD

#define OUTIN_MOTOR_CLK_PORT	GPIOD
#define OUTIN_MOTOR_CLK_PIN	GPIO_PIN_12
#define OUTIN_MOTOR_CLK_SRC	RCC_AHB1Periph_GPIOD

// OC for fix motor, PE2
#define FIX_OC_CLK_PORT	GPIOE
#define FIX_OC_CLK_PIN	GPIO_PIN_2
#define FIX_OC_CLK_SRC	RCC_AHB1Periph_GPIOE

// OC for out_in motor, PE3
#define OUT_OC_CLK_PORT	GPIOE
#define OUT_OC_CLK_PIN	GPIO_PIN_3
#define OUT_OC_CLK_SRC	RCC_AHB1Periph_GPIOE

// OC for out_in motor, PE4
#define IN_OC_CLK_PORT	GPIOE
#define IN_OC_CLK_PIN	GPIO_PIN_4
#define IN_OC_CLK_SRC	RCC_AHB1Periph_GPIOE



#endif


