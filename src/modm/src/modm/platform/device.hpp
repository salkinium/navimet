/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009, Thorsten Lajewski
 * Copyright (c) 2009-2010, 2016, Fabian Greif
 * Copyright (c) 2012-2013, 2016, 2018 Niklas Hauser
 * Copyright (c) 2013, Kevin Laeufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_DEVICE_HPP
#define MODM_DEVICE_HPP

#define STM32L432xx 1
#include <stdint.h>
// Defines for example the modm_always_inline macro
#include <modm/architecture/utils.hpp>

// Include external device headers:
#include <stm32l432xx.h>
#include <system_stm32l4xx.h>
/// @cond
// This is a hack to make the *_Typedef's known to GDB, so that you can debug
// the peripherals directly in GDB in any context.
// Otherwise GDB would throw a "no symbol 'GPIO_TypeDef' in current context".
extern ADC_TypeDef				___ADC1			;
extern ADC_Common_TypeDef		___ADC1_COMMON		;
extern CAN_TypeDef				___CAN				;
extern CAN_TypeDef				___CAN1			;
extern COMP_TypeDef			___COMP1			;
extern COMP_Common_TypeDef		___COMP12_COMMON	;
extern COMP_TypeDef			___COMP2			;
extern CRC_TypeDef				___CRC				;
extern CRS_TypeDef				___CRS				;
extern CoreDebug_Type			___CoreDebug		;
extern DAC_TypeDef				___DAC				;
extern DAC_TypeDef				___DAC1			;
extern DBGMCU_TypeDef			___DBGMCU			;
extern DMA_TypeDef				___DMA1			;
extern DMA_Request_TypeDef		___DMA1_CSELR		;
extern DMA_Channel_TypeDef		___DMA1_Channel1	;
extern DMA_Channel_TypeDef		___DMA1_Channel2	;
extern DMA_Channel_TypeDef		___DMA1_Channel3	;
extern DMA_Channel_TypeDef		___DMA1_Channel4	;
extern DMA_Channel_TypeDef		___DMA1_Channel5	;
extern DMA_Channel_TypeDef		___DMA1_Channel6	;
extern DMA_Channel_TypeDef		___DMA1_Channel7	;
extern DMA_TypeDef				___DMA2			;
extern DMA_Request_TypeDef		___DMA2_CSELR		;
extern DMA_Channel_TypeDef		___DMA2_Channel1	;
extern DMA_Channel_TypeDef		___DMA2_Channel2	;
extern DMA_Channel_TypeDef		___DMA2_Channel3	;
extern DMA_Channel_TypeDef		___DMA2_Channel4	;
extern DMA_Channel_TypeDef		___DMA2_Channel5	;
extern DMA_Channel_TypeDef		___DMA2_Channel6	;
extern DMA_Channel_TypeDef		___DMA2_Channel7	;
extern DWT_Type				___DWT				;
extern EXTI_TypeDef			___EXTI			;
extern FIREWALL_TypeDef		___FIREWALL		;
extern FLASH_TypeDef			___FLASH			;
extern FPU_Type				___FPU				;
extern GPIO_TypeDef			___GPIOA			;
extern GPIO_TypeDef			___GPIOB			;
extern GPIO_TypeDef			___GPIOC			;
extern GPIO_TypeDef			___GPIOH			;
extern I2C_TypeDef				___I2C1			;
extern I2C_TypeDef				___I2C3			;
extern ITM_Type				___ITM				;
extern IWDG_TypeDef			___IWDG			;
extern LPTIM_TypeDef			___LPTIM1			;
extern LPTIM_TypeDef			___LPTIM2			;
extern USART_TypeDef			___LPUART1			;
extern MPU_Type				___MPU				;
extern NVIC_Type				___NVIC			;
extern OPAMP_TypeDef			___OPAMP			;
extern OPAMP_TypeDef			___OPAMP1			;
extern OPAMP_Common_TypeDef	___OPAMP1_COMMON	;
extern PWR_TypeDef				___PWR				;
extern QUADSPI_TypeDef			___QUADSPI			;
extern RCC_TypeDef				___RCC				;
extern RNG_TypeDef				___RNG				;
extern RTC_TypeDef				___RTC				;
extern SAI_TypeDef				___SAI1			;
extern SAI_Block_TypeDef		___SAI1_Block_A	;
extern SAI_Block_TypeDef		___SAI1_Block_B	;
extern SCB_Type				___SCB				;
extern SCnSCB_Type				___SCnSCB			;
extern SPI_TypeDef				___SPI1			;
extern SPI_TypeDef				___SPI3			;
extern SWPMI_TypeDef			___SWPMI1			;
extern SYSCFG_TypeDef			___SYSCFG			;
extern SysTick_Type			___SysTick			;
extern TIM_TypeDef				___TIM1			;
extern TIM_TypeDef				___TIM15			;
extern TIM_TypeDef				___TIM16			;
extern TIM_TypeDef				___TIM2			;
extern TIM_TypeDef				___TIM6			;
extern TIM_TypeDef				___TIM7			;
extern TPI_Type				___TPI				;
extern TSC_TypeDef				___TSC				;
extern USART_TypeDef			___USART1			;
extern USART_TypeDef			___USART2			;
extern USB_TypeDef				___USB				;
extern WWDG_TypeDef			___WWDG			;
/// @endcond

#endif  // MODM_DEVICE_HPP