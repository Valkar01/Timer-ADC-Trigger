/*
 * stm32f411xx_Timer_driver.c
 *
 *  Created on: 07-Jan-2026
 *      Author: Karthik
 */

#include"stm32f411xx.h"

void Timer_Init(Timer_Handle_t *pTimer_Handle)
{
	uint16_t tempARR;
	TIM2_PCLK_EN;
	// 1. Set the CLK Period
	pTimer_Handle->pTIMx->PSC = pTimer_Handle->Timer_Config.Prescaler;
	tempARR = 16000000/(((pTimer_Handle->Timer_Config.Prescaler) + 1) * (pTimer_Handle->Timer_Config.Period));
	pTimer_Handle->pTIMx->ARR = tempARR;

	// 2. Trigger at update event
	pTimer_Handle->pTIMx->CR2 |= (2 << 4);

	// 3. Update event source
	pTimer_Handle->pTIMx->CR1 |= (UEV_OVRF_UNDF << 2);
}

void Timer_Start(Timer_2to5_Reg_Def_t *pTIMx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		pTIMx->CR1 |= (1 << 0);
	}
	else
	{
		pTIMx->CR1 &= ~(1 << 0);
	}
}
