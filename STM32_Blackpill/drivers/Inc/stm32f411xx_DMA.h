/*
 * stm32f411xx_DMA.h
 *
 *  Created on: 14-Jan-2026
 *      Author: Karthik
 */

#ifndef INC_STM32F411XX_DMA_H_
#define INC_STM32F411XX_DMA_H_

#include"stm32f411xx.h"

/*
 * Macros to set teh direction of data flow through DMA
 */
#define PERIPH_TO_MEM      0
#define MEM_TO_PERIPH      1
#define MEM_TO_MEM         2

#define BYTE               0
#define HALF_WORD          1
#define WORD               2

#define DMA_FLOW           0
#define PERI_FLOW          1

typedef struct
{
	uint8_t Src_Dest;
	uint32_t *Peri_Port_Addr;
	uint32_t *Memory_Addr;
	uint8_t Peri_Data_Size;
	uint8_t Flow_Control;
}DMA_Config_t;

typedef struct
{
	DMA_Config_t DMA_Config;
	DMA_Reg_Def_t *pDMAx;
}DMA_Handle_t;

void DMA_Config(DMA_Handle_t *pDMA_Handle);
void DMA_Flag_Reset(DMA_Reg_Def_t *pDMAx);
void DMA_Transfer(DMA_Reg_Def_t *pDMAx, uint8_t EnorDi);


#endif /* INC_STM32F411XX_DMA_H_ */
