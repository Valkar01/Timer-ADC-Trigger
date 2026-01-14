/*
 * stm32f411xx_timer_driver.h
 *
 *  Created on: 07-Jan-2026
 *      Author: Karthik
 */

#ifndef INC_STM32F411XX_TIMER_H_
#define INC_STM32F411XX_TIMER_H_

#include"stm32f411xx.h"

#define UEV_OVRF_UNDF       1

typedef struct
{
	uint16_t Prescaler;
	uint32_t Period;

}Timer_Config_t;

typedef struct
{
	Timer_2to5_Reg_Def_t *pTIMx;
	Timer_Config_t Timer_Config;
}Timer_Handle_t;

void Timer_Init(Timer_Handle_t *pTimer_Handle);
void Timer_Start(Timer_2to5_Reg_Def_t *pTIMx, uint8_t EnorDi);
#endif /* INC_STM32F411XX_TIMER_H_ */
