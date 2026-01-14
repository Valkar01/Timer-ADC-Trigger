/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: 06-Jan-2026
 *      Author: Karthik
 */

#include"stm32f411xx.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Enable the GPIO peripheral clock
		GPIOA_PCLK_EN;


	uint32_t temp = 0 ;
	//1. configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode <= GPIO_MODE_ANALOG)
	{
		 // temp register
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Mode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}

	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_OUT)
	{
		//2. configure the output type
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;

		//4. Configure the OP Type
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOptype << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		temp = 0;
	}

	//3. configure the pupd register
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//5. Configure alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_ALF)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_AltFuncMode << (4 * temp2);
	}
}
