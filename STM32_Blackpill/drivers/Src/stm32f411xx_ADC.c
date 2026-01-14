/*
 * stm32f411xx_ADC.c
 *
 *  Created on: 14-Jan-2026
 *      Author: Karthik
 */

#include"stm32f411xx.h"

void ADC_Init(ADC_Handle_t *pADC_Handle)
{
	ADC1_PCLK_EN;
	//1. Enable the external trigger for regular channel
	pADC_Handle->pADCx->CR2 |= (pADC_Handle->ADC_Config.External_Trigger << 28);

	//2. Select the External Trigger source
	pADC_Handle->pADCx->CR2 |= (pADC_Handle->ADC_Config.Extern_eve_select << 24);

	//3. Select the conversion mode
	pADC_Handle->pADCx->CR2 |= (pADC_Handle->ADC_Config.Conversion_Mode << 1);

	//4.Select the alignment of data in DR
	pADC_Handle->pADCx->CR2 |= (pADC_Handle->ADC_Config.Alignment << 11);

	//5. DMA request issued as long as conversion takes place
	pADC_Handle->pADCx->CR2 |= (1 << 9);

	//6. Enable DMA mode
	pADC_Handle->pADCx->CR2 |= (1 << 8);
}

void ADC_Start_Conversion(ADC_Reg_Def_t *pADCx, uint8_t EnorDi)
{
	//1. Enable the ADC
	if(EnorDi == ENABLE)
	{
		pADCx->CR2 |= (1 << 0);
	}
	else pADCx->CR2 &= ~(1 << 0);
}
