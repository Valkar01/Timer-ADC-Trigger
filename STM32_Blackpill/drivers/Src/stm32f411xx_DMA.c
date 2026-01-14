/*
 * stm32f411xx_DMA.c
 *
 *  Created on: 14-Jan-2026
 *      Author: Karthik
 */

#include"stm32f411xx.h"

void DMA_Config(DMA_Handle_t *pDMA_Handle)
{
	DMA2_PCLK_EN;
	//1. Check first the stream is disabled
	if(pDMA_Handle->pDMAx->S0CR & 1)
	{
		uint8_t dummy_rd;
		pDMA_Handle->pDMAx->S0CR &= ~(1 << 0);
		dummy_rd = pDMA_Handle->pDMAx->S0CR & 1;
		while(dummy_rd);
		DMA_Flag_Reset(pDMA_Handle->pDMAx);
	}

		//2. Set the peripheral port address
		pDMA_Handle->pDMAx->S0PAR = (uint32_t)pDMA_Handle->DMA_Config.Peri_Port_Addr;

		//3. Set the Memory address
		pDMA_Handle->pDMAx->S0M0AR = (uint32_t)pDMA_Handle->DMA_Config.Memory_Addr;

		//4. Configure the total No. of data items to be transfered
		pDMA_Handle->pDMAx->S0NDTR = 1;

		//5. Select the DMA channel
		pDMA_Handle->pDMAx->S0CR &= ~(7 << 25);

		//6. Consider the stream priority
		pDMA_Handle->pDMAx->S0CR |= (3 << 16);

		//7. Configure the data direction
		pDMA_Handle->pDMAx->S0CR |= (pDMA_Handle->DMA_Config.Src_Dest << 6);

		//8. Peripheral Data size
		pDMA_Handle->pDMAx->S0CR |= (pDMA_Handle->DMA_Config.Peri_Data_Size << 11);

		//9. Flow Control
		pDMA_Handle->pDMAx->S0CR |= (pDMA_Handle->DMA_Config.Flow_Control << 5);

		//10. Circular Mode
		pDMA_Handle->pDMAx->S0CR &= ~(1 << 8);
}

void DMA_Transfer(DMA_Reg_Def_t *pDMAx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pDMAx->S0CR |= (1 << 0);
	}
	else pDMAx->S0CR &= ~(1 << 0);
}

void DMA_Flag_Reset(DMA_Reg_Def_t *pDMAx)
{
	pDMAx->HIFCR &= ~(0x0F7D0F7D);
	pDMAx->LIFCR &= ~(0x0F7D0F7D);
}
