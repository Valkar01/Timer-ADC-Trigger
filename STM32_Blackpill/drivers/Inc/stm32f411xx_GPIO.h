/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: 06-Jan-2026
 *      Author: Karthik
 */

#ifndef INC_STM32F411XX_GPIO_H_
#define INC_STM32F411XX_GPIO_H_

#include"stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber ;
	uint8_t GPIO_Mode ;
	uint8_t GPIO_PinSpeed ;
	uint8_t GPIO_PinPuPdControl ;
	uint8_t GPIO_PinOptype ;
	uint8_t GPIO_AltFuncMode ;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_Reg_Def_t *pGPIOx ;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * GPIO Pin Numbers
 */
#define GPIO_P0                  0
#define GPIO_P1                  1
#define GPIO_P2                  2
#define GPIO_P3                  3
#define GPIO_P4                  4
#define GPIO_P5                  5
#define GPIO_P6                  6
#define GPIO_P7                  7
#define GPIO_P8                  8
#define GPIO_P9                  9
#define GPIO_P10                 10
#define GPIO_P11                 11
#define GPIO_P12                 12
#define GPIO_P13                 13
#define GPIO_P14                 14
#define GPIO_P15                 15

/*
 * GPIO pin Modes
 */
#define GPIO_MODE_IN             0
#define GPIO_MODE_OUT            1
#define GPIO_MODE_ALF            2
#define GPIO_MODE_ANALOG         3
#define GPIO_MODE_IT_FEDGE       4
#define GPIO_MODE_IT_REDGE       5
#define GPIO_MODE_IT_RFEDGE      6

/*
 * GPIO Pin Possible Output Types
 */
#define GPIO_OP_TYPE_PP          0
#define GPIO_OP_TYPE_OD          1

/*
 * GPIO Pin Possible Output Speeds
 */
#define GPIO_LOW_SPEED           0
#define GPIO_MEDIUM_SPEED        1
#define GPIO_FAST_SPEED          2
#define GPIO_HIGH_SPEED         3

/*
 * GPIO Pin PullUp and PullDown
 */
#define GPIO_NO_PD_PU            0
#define GPIO_PU                  1
#define GPIO_PD                  2

/*********************************************************
 * Some APIs supported by this driver
 *********************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PCLK_Ctrl(GPIO_Reg_Def_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);


#endif /* INC_STM32F411XX_GPIO_H_ */
