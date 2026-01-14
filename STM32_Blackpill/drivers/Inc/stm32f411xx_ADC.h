/*
 * stm32f411_ADC.h
 *
 *  Created on: 14-Jan-2026
 *      Author: Karthik
 */

#ifndef INC_STM32F411XX_ADC_H_
#define INC_STM32F411XX_ADC_H_

#include"stm32f411xx.h"

#define EXTERN_TRIG_DI        0
#define EXTERN_TRIG_REDGE     1
#define EXTERN_TRIG_FEDGE     2
#define EXTERN_TRIG_RFEDGE    3

#define TIM2_TRGO_EVENT       6

#define DMA_MODE_DI           0
#define DMA_MODE_EN           1

#define SINGLE_MODE		      0
#define CONTINOUS_MODE		  1

#define RIGHT_ALIGN           0
#define LEFT_ALIGN            1

typedef struct
{
	uint8_t External_Trigger;
	uint8_t Extern_eve_select;
	uint8_t Conversion_Mode;
	uint8_t Alignment;
}ADC_Config_t;

typedef struct
{
	ADC_Config_t ADC_Config;
	ADC_Reg_Def_t *pADCx;
}ADC_Handle_t;

void ADC_Init(ADC_Handle_t *pADC_Handle);

void ADC_Start_Conversion(ADC_Reg_Def_t *pADCx, uint8_t EnorDi);

#endif /* INC_STM32F411XX_ADC_H_ */
